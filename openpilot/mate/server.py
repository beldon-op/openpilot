"""
mate — 车机事件实时推送服务（局域网）

把 poller.py 推导的四类边沿事件（events / opstate / button / sound）通过
WebSocket 和 SSE 推给局域网客户端。参考实现为 cp_byd local_collector，
其中 admin 管理 API 与 10Hz 周期帧均不移植。

接口：
  GET /ws                 WebSocket（heartbeat 20s）
  GET /stream             SSE（?kinds=events,sound 可过滤）
  GET /state              一次性状态快照（car/opstate/events/sound 最新值，不周期推流）
  GET /health             健康检查（不鉴权）
  GET /sounds/<file.wav>  播放 sound 事件对应的音效文件（随语言目录切换）

WS 客户端也可发 {"type":"snapshot"} 走同一条连接拿快照。协议文档见 mate/API.md。

可选鉴权：params 写 MateToken 后，/ws /stream /state /sounds 必须带 ?token=xxx 或
Authorization: Bearer xxx。白名单外的 key 走 /data/params/d 文件直读。

注册方式（system/manager/process_config.py）：
  PythonProcess("mate", "openpilot.mate.server", always_run, restart_if_crash=True)
"""
import asyncio
import datetime
import hmac
import json
import os
import threading
import time

from aiohttp import web, WSMsgType

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.common.swaglog import cloudlog
from openpilot.mate.poller import MatePoller
from openpilot.selfdrive.ui.soundd import (
  read_sound_language_setting,
  sound_asset_dir_for_language,
)

PORT = int(os.environ.get("MATE_PORT", "8083"))
ALL_KINDS = ("events", "opstate", "button", "sound")
STATE_KINDS = ("events", "opstate", "sound")  # emit 时同步写进 /state 快照的类型
REPLAY_KINDS = ("events", "sound")  # 新连接先补发这两类的当前状态
TOKEN_PARAM = "MateToken"
SLOW_CLIENT_QUEUE = 256
SSE_KEEPALIVE_S = 15.0

ASSETS_DIR = os.path.join(BASEDIR, "openpilot", "selfdrive", "assets")


def wallclock() -> float:
  """Unix 墙钟秒。time.time 被仓库 lint 禁用（monotonic 才是进程内计时），
  快照 ts 是跨机对时语义，必须墙钟；设备时钟错时客户端自行对齐。"""
  return datetime.datetime.now(datetime.UTC).timestamp()

_token_lock = threading.Lock()
_token_cache: tuple[float, str | None] = (0.0, None)


def read_token() -> str | None:
  """读取鉴权 token，1 秒缓存；Params 对白名单外 key 抛 UnknownKeyName，
  回落到直读 params 目录文件（本仓多个 daemon 同样做法）。"""
  global _token_cache
  with _token_lock:
    ts, cached = _token_cache
    if time.monotonic() - ts < 1.0:
      return cached
    val = None
    try:
      got = Params().get(TOKEN_PARAM)
      val = got.decode("utf-8", "ignore") if isinstance(got, (bytes, bytearray)) else str(got or "") or None
    except Exception:
      try:
        with open(os.path.join(Params().get_param_path(), TOKEN_PARAM), "rb") as f:
          val = f.read().decode("utf-8", "ignore") or None
      except OSError:
        val = None
    _token_cache = (time.monotonic(), val)
    return val


def resolve_wav_path(filename: str) -> str | None:
  """随语言目录切换解析音效文件，缺失时回退英文目录（与 soundd 加载逻辑一致）"""
  lang = read_sound_language_setting(Params())
  path = os.path.join(ASSETS_DIR, sound_asset_dir_for_language(lang), filename)
  if os.path.isfile(path):
    return path
  path = os.path.join(ASSETS_DIR, "sounds_eng", filename)
  return path if os.path.isfile(path) else None


def check_auth(request: web.BaseRequest) -> None:
  token = read_token()
  if not token:
    return
  auth = request.headers.get("Authorization", "")
  provided = request.query.get("token") or (auth[7:] if auth.startswith("Bearer ") else "")
  if not hmac.compare_digest(provided, token):
    raise web.HTTPUnauthorized(text='{"error":"invalid token"}', content_type="application/json")


class Hub:
  """poller 线程 -> 事件循环 -> 各客户端队列的广播中心，并维护最新状态快照。"""

  def __init__(self):
    self.loop: asyncio.AbstractEventLoop | None = None
    self.clients: dict[asyncio.Queue, frozenset[str]] = {}
    self.replay: dict[str, str] = {}  # kind -> 最新 envelope（供新连接补发）
    self.snapshot: dict[str, dict] = {}  # kind -> 最新 data（GET /state 用）
    self._snapshot_lock = threading.Lock()
    self.start_time = time.monotonic()

  def add_client(self, kinds) -> asyncio.Queue:
    q: asyncio.Queue = asyncio.Queue(maxsize=SLOW_CLIENT_QUEUE)
    self.clients[q] = frozenset(kinds)
    return q

  def remove_client(self, q: asyncio.Queue) -> None:
    self.clients.pop(q, None)

  def set_state(self, kind: str, data: dict) -> None:
    """非边沿状态（car 快照）只进快照，不广播、不补发。每段带自己的写入时间。"""
    with self._snapshot_lock:
      self.snapshot[kind] = {"ts": wallclock(), "data": data}

  def emit(self, kind: str, data: dict) -> None:
    """任意线程可调。events/sound 同时维护补发快照；STATE_KINDS 进 /state。"""
    payload = json.dumps({"type": kind, "data": data}, ensure_ascii=False)
    if kind in STATE_KINDS:
      self.set_state(kind, data)
    if kind in REPLAY_KINDS:
      active = kind == "events" and data.get("active")
      playing = kind == "sound" and data.get("sound", "none") != "none"
      if active or playing:
        self.replay[kind] = payload
      else:
        self.replay.pop(kind, None)
    if self.loop is None:
      return
    self.loop.call_soon_threadsafe(self._broadcast, kind, payload)

  def _broadcast(self, kind: str, payload: str) -> None:
    for q, kinds in list(self.clients.items()):
      if kind not in kinds:
        continue
      try:
        if q.full():  # 慢客户端丢最旧（事件频率很低，正常不会触发）
          q.get_nowait()
        q.put_nowait((kind, payload))
      except (asyncio.QueueEmpty, RuntimeError):
        self.remove_client(q)

  def replay_snapshot(self) -> list[str]:
    return [self.replay[k] for k in REPLAY_KINDS if k in self.replay]

  def state_snapshot(self) -> dict:
    """当前全量状态快照；未收到过的段不出现（客户端据此判断数据新鲜度）"""
    with self._snapshot_lock:
      return {"ts": datetime.datetime.now(datetime.UTC).timestamp(), "data": dict(self.snapshot)}


hub = Hub()


async def ws_handler(request: web.Request):
  check_auth(request)
  ws = web.WebSocketResponse(heartbeat=20)
  await ws.prepare(request)

  kinds = set(ALL_KINDS)
  q = hub.add_client(kinds)
  cloudlog.event("mate ws connected", n=len(hub.clients))
  await ws.send_str(json.dumps({
    "type": "hello",
    "data": {"service": "mate", "version": 1, "port": PORT, "kinds": list(ALL_KINDS)},
  }, ensure_ascii=False))
  for payload in hub.replay_snapshot():
    await ws.send_str(payload)

  async def pump():
    while True:
      _, payload = await q.get()
      await ws.send_str(payload)

  pump_task = asyncio.create_task(pump())
  try:
    async for msg in ws:
      if msg.type != WSMsgType.TEXT:
        continue
      try:
        req = json.loads(msg.data)
      except json.JSONDecodeError:
        continue
      rtype = req.get("type")
      if rtype == "subscribe":
        picked = {k for k in req.get("kinds", []) if k in ALL_KINDS}
        kinds.clear()
        kinds.update(picked or ALL_KINDS)
        await ws.send_str(json.dumps({"type": "subscribed", "data": {"kinds": sorted(kinds)}}))
      elif rtype == "snapshot":
        await ws.send_str(json.dumps({"type": "snapshot", "data": hub.state_snapshot()}, ensure_ascii=False))
      elif rtype == "ping":
        await ws.send_str(json.dumps({"type": "pong", "data": {"ts": time.monotonic()}}))
      # 未知消息忽略
  except (ConnectionResetError, RuntimeError):
    pass
  finally:
    pump_task.cancel()
    hub.remove_client(q)
    cloudlog.event("mate ws disconnected", n=len(hub.clients))
  return ws


async def sse_handler(request: web.Request):
  check_auth(request)
  picked = {k for k in request.query.get("kinds", "").split(",") if k in ALL_KINDS}
  q = hub.add_client(picked or ALL_KINDS)

  resp = web.StreamResponse(headers={
    "Content-Type": "text/event-stream",
    "Cache-Control": "no-cache",
    "Connection": "keep-alive",
    "X-Accel-Buffering": "no",
  })
  await resp.prepare(request)
  cloudlog.event("mate sse connected", n=len(hub.clients))
  try:
    await resp.write(b": connected\n\n")
    for payload in hub.replay_snapshot():
      kind = json.loads(payload)["type"]
      await resp.write(f"event: {kind}\ndata: {payload}\n\n".encode())
    while True:
      try:
        kind, payload = await asyncio.wait_for(q.get(), timeout=SSE_KEEPALIVE_S)
      except TimeoutError:
        await resp.write(b": ping\n\n")
        continue
      await resp.write(f"event: {kind}\ndata: {payload}\n\n".encode())
  except (ConnectionResetError, BrokenPipeError, asyncio.CancelledError):
    pass
  finally:
    hub.remove_client(q)
  return resp


async def sounds_handler(request: web.Request):
  """GET /sounds/<file>.wav：sound 事件的配套音效，目录逻辑与 soundd 一致"""
  check_auth(request)
  filename = request.match_info.get("filename", "")
  if filename != os.path.basename(filename) or not filename.endswith(".wav"):
    raise web.HTTPNotFound(text='{"error":"sound not found"}', content_type="application/json")
  path = await asyncio.to_thread(resolve_wav_path, filename)
  if path is None:
    raise web.HTTPNotFound(text='{"error":"sound not found"}', content_type="application/json")
  return web.FileResponse(path)


async def state_handler(request: web.Request):
  """GET /state：一次性全量快照（car/opstate/events/sound 最新值 + 各自 ts）。
  未收到过的段不出现；car 段在 offroad 后保持最后一次 onroad 状态并随 ts 变旧。"""
  check_auth(request)
  return web.json_response(hub.state_snapshot())


async def health_handler(request: web.Request):
  return web.json_response({
    "ok": True,
    "service": "mate",
    "uptime": round(time.monotonic() - hub.start_time, 1),
    "clients": len(hub.clients),
    "token_auth": read_token() is not None,
  })


async def _bind_loop(_app: web.Application) -> None:
  hub.loop = asyncio.get_running_loop()


def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("mate: failed to set core affinity")

  threading.Thread(target=MatePoller(hub.emit, hub.set_state).run, name="mate-poller", daemon=True).start()

  app = web.Application()
  app.on_startup.append(_bind_loop)
  app.add_routes([
    web.get("/ws", ws_handler),
    web.get("/stream", sse_handler),
    web.get("/state", state_handler),
    web.get("/health", health_handler),
    web.get("/sounds/{filename}", sounds_handler),
  ])
  cloudlog.event("mate listening", port=PORT)
  web.run_app(app, host="0.0.0.0", port=PORT, print=None)


if __name__ == "__main__":
  main()

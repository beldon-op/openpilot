"""mate server params 读写端点测试（v5/v6 双向控制）。

按本 fork design D3 收敛后的白名单（8 键，均 offroad_only=False）测试：
- 读：白名单内 key 回 BOOL→布尔 / INT→整数；白名单外 403
- 写：类型归一（bool/int）+ 距离参数界限（0..2000）；白名单外 403；
  MateToken 设了 token 必须验（401）
- 响应形状对齐 CarrotPilot 7000 carrot_server 契约（carrotmate 端同一解析）

环境无 pytest-asyncio/anyio 插件，async 测试体经 asyncio.run() 同步包装。
"""
import asyncio

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.common.params import Params
from openpilot.mate import server


def _app() -> web.Application:
  app = web.Application()
  app.add_routes([
    web.get("/params", server.params_read_handler),
    web.post("/param_set", server.param_set_handler),
  ])
  return app


@pytest.fixture
def params_dir(tmp_path):
  p = Params(str(tmp_path))
  original = server.Params
  server.Params = lambda: p
  yield p
  server.Params = original


async def _client() -> TestClient:
  client = TestClient(TestServer(_app()))
  await client.start_server()
  return client


def _run(coro):
  return asyncio.run(coro)


class TestParamsRead:
  def test_read_bool_and_int(self, params_dir):
    params_dir.put("NaviGuidedLaneChangeEnabled", True, block=True)
    params_dir.put("NaviLaneChangeTurnDistance", 400, block=True)

    async def body():
      client = await _client()
      try:
        resp = await client.get("/params", params={"names": "NaviGuidedLaneChangeEnabled,NaviLaneChangeTurnDistance"})
        assert resp.status == 200
        data = await resp.json()
        assert data["ok"] is True
        assert data["values"]["NaviGuidedLaneChangeEnabled"] is True
        assert data["values"]["NaviLaneChangeTurnDistance"] == 400
      finally:
        await client.close()

    _run(body())

  def test_read_whitelist_rejected(self, params_dir):
    async def body():
      client = await _client()
      try:
        resp = await client.get("/params", params={"names": "DongleId"})
        assert resp.status == 403
      finally:
        await client.close()

    _run(body())

  def test_read_missing_names_400(self, params_dir):
    async def body():
      client = await _client()
      try:
        resp = await client.get("/params")
        assert resp.status == 400
      finally:
        await client.close()

    _run(body())


class TestParamSet:
  def test_set_bool(self, params_dir):
    async def body():
      client = await _client()
      try:
        resp = await client.post("/param_set", json={"name": "NaviGuidedLaneChangeEnabled", "value": True})
        assert resp.status == 200
        data = await resp.json()
        assert data["ok"] is True
        assert data["value"] is True
        assert params_dir.get_bool("NaviGuidedLaneChangeEnabled") is True
      finally:
        await client.close()

    _run(body())

  def test_set_bool_string_normalized(self, params_dir):
    # 兼容 carrotmate 端 value:"true"|"false"|1|0 的请求形态
    async def body():
      client = await _client()
      try:
        for v in ("true", "1", True):
          resp = await client.post("/param_set", json={"name": "ExperimentalMode", "value": v})
          assert resp.status == 200
          assert (await resp.json())["value"] is True
        for v in ("false", "0", False):
          resp = await client.post("/param_set", json={"name": "ExperimentalMode", "value": v})
          assert resp.status == 200
          assert (await resp.json())["value"] is False
      finally:
        await client.close()

    _run(body())

  def test_set_int_clamps_range(self, params_dir):
    async def body():
      client = await _client()
      try:
        resp = await client.post("/param_set", json={"name": "NaviLaneChangeTurnDistance", "value": 5000})
        assert resp.status == 400
        resp = await client.post("/param_set", json={"name": "NaviLaneChangeTurnDistance", "value": 250})
        data = await resp.json()
        assert data["ok"] is True and data["value"] == 250
      finally:
        await client.close()

    _run(body())

  def test_set_whitelist_rejected(self, params_dir):
    async def body():
      client = await _client()
      try:
        resp = await client.post("/param_set", json={"name": "DongleId", "value": "x"})
        assert resp.status == 403
        # sunnypilot 独有 param 在本 fork 白名单外
        resp = await client.post("/param_set", json={"name": "Mads", "value": True})
        assert resp.status == 403
      finally:
        await client.close()

    _run(body())

  def test_set_requires_token_when_set(self, params_dir, monkeypatch):
    monkeypatch.setattr(server, "read_token", lambda: "secret")

    async def body():
      client = await _client()
      try:
        body_json = {"name": "CarrotRouteCurvatureEnabled", "value": True}
        resp = await client.post("/param_set", json=body_json)
        assert resp.status == 401
        resp = await client.post("/param_set", json=body_json, params={"token": "secret"})
        assert resp.status == 200
        assert (await resp.json())["value"] is True
      finally:
        await client.close()

    _run(body())


class TestWhitelistShape:
  def test_whitelist_covers_carrot_params(self):
    # design D3 白名单 8 键必须全部在位（防端到端断链）
    for key in ("NaviGuidedLaneChangeEnabled", "NaviLaneChangeDirectionEnabled",
                "NaviLaneChangeTurnDistance", "NaviLaneChangeCrossroadDistance",
                "NaviLaneChangeSignalDistance", "NaviLaneChangeDirectionDistance",
                "CarrotRouteCurvatureEnabled", "ExperimentalMode"):
      assert key in server.PARAMS_WHITELIST, f"{key} 应在白名单内"

  def test_sp_only_keys_excluded(self):
    # design D3 剔除：sunnypilot 独有 param 不得混入本 fork 白名单
    for key in ("Mads", "AutoLaneChangeTimer", "SpeedLimitMode", "SmartCruiseControlMap",
                "LongitudinalPersonality", "LaneTurnValue"):
      assert key not in server.PARAMS_WHITELIST, f"{key} 不应在白名单内"

  def test_v5_keys_remain_anytime_writable(self):
    # 当前 8 键均 offroad_only=False（沿用 v5 兼容既有通道语义）
    for key in ("NaviGuidedLaneChangeEnabled", "NaviLaneChangeDirectionEnabled",
                "NaviLaneChangeTurnDistance", "NaviLaneChangeCrossroadDistance",
                "NaviLaneChangeSignalDistance", "NaviLaneChangeDirectionDistance",
                "CarrotRouteCurvatureEnabled", "ExperimentalMode"):
      assert server.PARAMS_WHITELIST[key][3] is False

  def test_no_credential_like_keys(self):
    # 凭据类键永远不得进入白名单（写通道只对控行为）
    for key in ("MateToken", "AccessToken", "GithubSshKeys", "SecOCKey", "DongleId"):
      assert key not in server.PARAMS_WHITELIST


if __name__ == "__main__":
  pytest.main([__file__])

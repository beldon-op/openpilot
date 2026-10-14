# Carrot Navi 7714 接口文档(设备接收端)

> 本文描述设备侧 `carrot_navi` 服务在 TCP 7714 上暴露的 HTTP / WebSocket 接口,
> 依据 `openpilot/selfdrive/carrot/carrot_navi.py` 的实现整理。
> 发送端(安卓 TMAP App)的完整协议契约(含各 JSON value 字段表)见
> `openpilot/selfdrive/carrot/carrot_navi_api.md`,本文不重复该文档的发送端细节。
> 7714 数据如何参与减速控制见 `carrot_navi_7713_7714_deceleration.md` 与
> `carrot_navi_7714_branch_comparison.md`。

## 1. 服务概览

- **进程**:`openpilot/system/manager/process_config.py` 中常驻进程 `carrot_navi`
  (`PythonProcess("carrot_navi", "openpilot.selfdrive.carrot.carrot_navi", always_run, restart_if_crash=True)`),
  永久独占 TCP 7714。端口被占时最多重试 10 次、间隔 0.5s。
- **职责**:接收车机内导航 App(TMAP)经 WebSocket 推送的导航结构化数据 / 图片 / 地图渲染流,
  校验后通过 cereal 发布 `carrotNavi`、`carrotNaviMedia` 消息供仪表簇等下游消费。
- **协议版本**:WebSocket v2(`protocol_version=2`,`catalog_revision=1`),固定 28 条流目录。
- **安全边界**:局域网明文 `ws://`,无 TLS / 鉴权 / 配对令牌。

| 端口 | 协议 | 方向 | 用途 |
| --- | --- | --- | --- |
| `7714` | TCP(HTTP + WebSocket) | 双向 | 控制协商、JSON / 图片 / 地图渲染流、健康查询 |
| `7705` | UDP 广播 | 设备 → App | 接收端发现信标(每 1s 广播本机 IP) |

其他常量:

| 常量 | 值 | 说明 |
| --- | --- | --- |
| `MAX_MESSAGE_BYTES` | 8 MiB | 单条 WebSocket 消息(含 CNV2 头)上限 |
| 控制通道心跳 | 5s | aiohttp 服务端 ping 周期 |
| 数据流通道心跳 | 10s | aiohttp 服务端 ping 周期 |
| 对端标识 | `X-Forwarded-For` 头优先,否则 remote 地址 | 记录在 health / latest 的 `peer` |

## 2. 发现机制(UDP 7705)

`CarrotNaviDiscoveryBeacon` 后台线程每 1s 向本机所有非回环 IPv4 接口的广播地址发送:

```json
{"ip": "192.168.1.33", "navi_debug": 1}
```

App 从该报文读取 `ip` 作为 7714 的 WebSocket host。`--advertise-ip` 可强制只通告指定地址,
`--no-beacon` 关闭信标。

## 3. HTTP 接口

### 3.1 `GET /` — 服务索引

```json
{
  "name": "Carrot Navi Receiver",
  "port": 7714,
  "protocol_version": 2,
  "health": "/health",
  "latest": "/api/navi/latest",
  "ws_control": "/api/navi/ws/v2/control/{version}",
  "ws_json": "/api/navi/ws/v2/json/{session_id}/{name}",
  "ws_image": "/api/navi/ws/v2/image/{session_id}/{name}",
  "ws_render": "/api/navi/ws/v2/render/{session_id}/{name}"
}
```

### 3.2 `GET /health` — 健康快照

返回(节选,完整以实现为准):

| 字段 | 说明 |
| --- | --- |
| `ok` / `service` / `port` | 固定 `true` / `carrot_navi_receiver` / 7714 |
| `protocol_version` | `2` |
| `map_theme` / `map_type` / `map_hz` / `map_bitrate_kbps` / `screen_center_y_ratio` | 当前生效地图配置 |
| `app_foreground` | App 前台布尔值;取自 `json:app_status`,未知时为 `null` |
| `control_connected` / `control_connections` | 控制通道是否已协商 / 连接数 |
| `session_id` / `app_version` / `manifest_revision` | 当前会话(App 版本来自控制 URL 的 `{version}` 段) |
| `received_count` / `session_received_count` | 累计 / 本次会话收到的消息数 |
| `last_received_at_ms` / `peer` / `error` | 最近收包时间、对端、最近错误 |
| `state_generation` | 状态代数,每次 JSON/配置变化 +1 |
| `cereal_publish_count` / `cereal_error` | cereal 发布计数与错误 |
| `items` | 每个 key(如 `json:speed`)的最新记录摘要:`kind`、`name`、`sequence`、`present`、`reason`、`peer`、`received_at_ms`、`bytes`;JSON 项另含 `sent_at_ms`、`value`,二进制项另含 `message_type`、`format_or_reason`、`flags`、`width`、`height` |

### 3.3 `GET /api/navi/latest` — 最近数据

结构同 `health.items`,外加 `last_control_events`(最近 20 条控制事件原文)。用于调试,不作为消费接口。

## 4. WebSocket 接口

App 是 WebSocket 客户端。四个端点:

| 端点 | App 发送的帧类型 | 用途 |
| --- | --- | --- |
| `/api/navi/ws/v2/control/{app_version}` | JSON 文本 | 协商与会话管理;`{app_version}` 为任意版本串 |
| `/api/navi/ws/v2/json/{session_id}/{name}` | JSON 文本 | 单条 JSON 数据流 |
| `/api/navi/ws/v2/image/{session_id}/{name}` | 二进制 | 单条 PNG 图片流(PNG 帧 + CLEAR) |
| `/api/navi/ws/v2/render/{session_id}/{name}` | 二进制 | `map_main` 渲染流(JPEG / H.264 + CLEAR) |

连接时序:

1. App 经 UDP 7705 发现拿到设备 IP,连接控制端点并发送 `requirements_query`。
2. 接收端校验后回 `subscription_manifest`(28 条流,含禁用项)。
3. App 打开每条启用流的独立 WebSocket,按 URL 携带 `session_id` 推流。
4. 会话由 `session_id` 绑定;URL 中 session 与当前会话不符即拒绝(`stale v2 session`)。

**单 owner 规则**:控制端点同一时刻只允许一个已协商连接。第二个手机(或重连)发起
`requirements_query` 时,若已有 owner 存活,接收端回
`protocol_error(code="receiver_busy")` 并以关闭码 **1013** 拒绝,保护原发送方的会话不被作废。
owner 断开后,下一个连接才能重新协商(生成新 `session_id`,旧会话立即失效)。

## 5. 控制通道消息

所有控制帧必须是 JSON 文本对象。除协商外的事件要求 `protocol_version=2`,否则回
`protocol_error(code="invalid_control_message")`;错误会记录到 `health.error`。

### 5.1 `requirements_query`(App → 接收端)

校验要点:

- `type == "requirements_query"`、`protocol_version == 2`、`catalog_revision == 1`。
- `streams` 必须是完整 28 项目录(与接收端目录逐项一致,`schema_version=1`),多 / 少 / 重名 / 重 handle 均拒绝。

### 5.2 `subscription_manifest`(接收端 → App)

协商成功后返回,`session_id` 为 16 位 hex 随机串,`revision` 固定为 `1`
(本接收端用新 `session_id` 表达会话更替,不在会话内递增 revision)。

各流 `params` 取值:

| 流类型 | params |
| --- | --- |
| json(13 项) | `delivery_mode=on_change`、`interval_ms=1000`、`stale_timeout_ms=10000` |
| image(14 项) | `format=png`、`max_fps=5`、`stale_timeout_ms=15000`;两个 `crossroad_*` 项追加 `theme=<map_theme>` |
| render(`map_main`) | 见下表 |

render params(随地图配置动态生成):

`composition=map_route_vehicle`、`width=960`、`height=540`、`dpi=360`、`fps=<map_hz>`、
`jpeg_quality=75`、`codec=h264`、`h264_bitrate_kbps=<自动计算>`、`h264_keyframe_interval_sec=2`、
`camera_mode=app_sync`、`map_theme=<theme>`、`map_type=<type>`、`zoom=11.0`、`tilt=50.0`、
`bearing=0.0`、`follow_vehicle_bearing=true`、`fov=40.0`、`screen_center_y_ratio=<比例>`、
`follow_vehicle=true`、`center_latitude=null`、`center_longitude=null`、`stale_timeout_ms=5000`。

### 5.3 启用策略

28 项中默认启用 27 项:全部 13 条 JSON、14 条图片中除 `lane_top` 外的 13 条、`map_main` 渲染流。
向禁用流的 URL 建立连接会被拒绝(`v2 stream is not enabled`)。

### 5.4 其余控制事件

协商之后控制通道上收到的其他 JSON(`manifest_applied`、`resource_status`、`stream_status`、
`protocol_error` 等)按原样存入事件环(供 `/api/navi/latest` 观察);`type=protocol_error`
时其 `message`/`code` 记为最近错误。不存在的 `type` 不主动回错(避免错误风暴)。

## 6. JSON 数据流

每条 JSON 流一个独立 WebSocket,帧为 JSON 文本对象:

```json
{
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "9f73c7b8c3132f10",
  "manifest_revision": 1,
  "schema_version": 1,
  "kind": "json",
  "name": "speed",
  "stream_handle": 6,
  "sequence": 91,
  "source_timestamp_ms": 1783920000200,
  "sent_at_ms": 1783920000201,
  "present": true,
  "value": {}
}
```

JSON 流目录(13 项):`vehicle`、`guidance_current`、`guidance_next`、`lane_current`、
`lane_ahead`、`speed`、`traffic_signal`、`crossroad`、`route`、`navigation_status`、
`app_status`、`camera_state`、`composition_state`。
value 字段表见 `carrot_navi_api.md` §5.2。

校验规则(违反任一条 → 回 `protocol_error(code="json_stream_error")` 并以 **1008** 关闭):

| 规则 | 说明 |
| --- | --- |
| 信封一致性 | `type=item_update`、`protocol_version=2`、`session_id` 与 URL 且与当前会话一致、`kind=json`、`name` 与 URL 一致、`manifest_revision`/`stream_handle`/`schema_version` 与 manifest 一致 |
| `sequence` | 非负整数,且每条流严格递增;回退或重复即拒绝(`stale v2 sequence`) |
| `source_timestamp_ms` / `sent_at_ms` | 非负整数 |
| `present=true` | `value` 必须为对象;`lane_ahead` 例外,必须是对象数组 |
| `present=false` | `value` 必须为 `null`,且带非空、≤64 字符的 `reason` 字符串(墓碑消息) |

cereal 侧只消费其中 10 项:`vehicle`、`guidance_current`、`guidance_next`、`lane_current`、
`lane_ahead`、`speed`、`traffic_signal`、`crossroad`、`route`、`navigation_status`
(`app_status` 仅用于前台判定,`camera_state`/`composition_state` 不进 cereal)。

## 7. 二进制数据流(image / render)

每条二进制消息 = **40 字节大端 CNV2 头 + payload**,与 APK 侧协议一致:

| 偏移 | 大小 | 字段 |
| ---: | ---: | --- |
| 0 | 4 | 魔数 `CNV2` |
| 4 | 1 | `protocol_version`,固定 2 |
| 5 | 1 | `message_type` |
| 6 | 1 | `format_or_reason` |
| 7 | 1 | `flags` |
| 8 | 4 | `stream_handle`(uint32,须与 manifest 一致且 >0) |
| 12 | 4 | `manifest_revision`(uint32,须与 manifest 一致且 >0) |
| 16 | 8 | `sequence`(int64,每流严格递增) |
| 24 | 8 | `source_timestamp_ms`(int64) |
| 32 | 4 | `payload_length`(uint32,必须等于实际 payload 长度) |
| 36 | 2 | `width`(uint16) |
| 38 | 2 | `height`(uint16) |

对应 Python 格式:`struct.Struct(">4sBBBBIIQQIHH")`。

消息类型与格式约束:

| `message_type` | 含义 | image 流 | render 流 |
| --- | --- | --- | --- |
| 1 | 图片帧 | format 必须 1(PNG,校验 `\x89PNG\r\n\x1a\n` 魔数),`width/height > 0` | format 必须 2(JPEG,校验 `FFD8…FFD9`) |
| 2 | 视频配置 | 不允许 | format 必须 3,Annex-B 起始码(`00000001`/`000001`);接收端缓存为该流最新配置 |
| 3 | 视频访问单元 | 不允许 | format 必须 3,Annex-B;`flags & 1`(KEYFRAME)时缓存为最新关键帧 |
| 4 | CLEAR(墓碑) | 允许 | 允许;payload 必须为空、`width=height=0` |

CLEAR 原因码(`format_or_reason`):`1=source_absent`、`2=cleared`、`3=expired`、`4=passed`、
`5=invalid`。

违反任一规则(魔数 / 长度 / 格式 / 尺寸 / 流身份 / 序列)→ 回
`protocol_error(code="binary_stream_error")` 并以 **1008** 关闭。非二进制帧直接拒绝。

## 8. 地图配置热更新

接收端每 1s 轮询 Params,`ClusterNaviMapParamReader` 读取:

| Param 键 | 取值 | 默认 |
| --- | --- | --- |
| `ClusterNaviMapTheme` | 0=auto / 1=dark / 2=light | 1(dark) |
| `ClusterNaviMapType` | 0=normal / 1=satellite | 0 |
| `ClusterNaviMapFps` | 档位 0/1/2/3 → 5/10/20/30 Hz | 1(10 Hz) |
| `CarrotNaviHudMapProfile` | 1=HUD 版式(`screen_center_y_ratio=0.68`) | 0(普通,0.80) |

码率自动计算:以 960×540@10Hz=3000kbps 为基准,按分辨率与帧率等比缩放
(5Hz→1500、20Hz→3000、30Hz→6000),钳制到 1..12000 kbps。

任一配置变化后,接收端**主动关闭全部 WebSocket(关闭码 1012,message `map configuration changed`)**,
App 需重连并重新协商(新 session,manifest 带新地图参数)。

## 9. 关闭码汇总

| 关闭码 | 场景 |
| --- | --- |
| 1008 | JSON / 二进制数据流校验失败(`invalid JSON stream` / `invalid binary stream`) |
| 1012 | 地图配置变化,要求重连重协商 |
| 1013 | 已有控制 owner,`receiver_busy` |

## 10. cereal 输出

接收端校验通过的数据由 `CarrotNaviCerealPublisher` 转发:

- `carrotNavi`:10 项簇内导航 JSON 的规范化快照(见 §6),状态变化触发。
- `carrotNaviMedia`:render 流的视频配置 / 关键帧及图片引导帧,供 web 端 late-join 引导
  (bootstrap 请求经 `CarrotNaviWebBootstrapRequest` 参数下发)。

发布计数与错误可在 `/health` 的 `cereal_publish_count` / `cereal_error` 观察。

## 11. 命令行参数

```
--host           监听地址,默认 0.0.0.0
--port           监听端口,默认 7714
--advertise-ip   发现信标通告的 IP(默认自动探测)
--no-beacon      关闭 UDP 7705 信标
--map-theme      覆盖地图主题(auto/dark/light),优先于 Params
--map-type       覆盖地图类型(normal/satellite)
--no-cereal      不启动 cereal 发布器
```

## 12. 调试速查

```sh
# 设备上(或 PC 上用设备 IP)
curl -s http://127.0.0.1:7714/health | python3 -m json.tool
curl -s http://127.0.0.1:7714/api/navi/latest | python3 -m json.tool

# 只关心某条流
curl -s http://127.0.0.1:7714/health | python3 -c "import json,sys; print(json.dumps(json.load(sys.stdin)['items'].get('json:speed'), indent=2))"
```

## 13. 参考

- 接收端实现:`openpilot/selfdrive/carrot/carrot_navi.py`
- 接收端测试:`openpilot/selfdrive/carrot/tests/test_carrot_navi.py`
- 发送端(APK)完整协议:`openpilot/selfdrive/carrot/carrot_navi_api.md`
- 减速控制侧:`docs/carrot_navi_7713_7714_deceleration.md`、`docs/carrot_navi_7714_branch_comparison.md`
- cereal payload:`openpilot/selfdrive/carrot/carrot_navi_cereal.py`

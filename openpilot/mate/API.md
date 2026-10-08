# mate 接口文档

车机事件实时推送服务。订阅 cereal 消息总线，把告警事件、openpilot 状态机跳转、
方向盘按键、播报声音四类**边沿事件**通过 **WebSocket** 或 **SSE** 推给局域网客户端，
并提供一次性状态快照 `GET /state`。

参考实现是 cp_byd 的 `local_collector/collector.py`；mate 是其事件子集——
**没有** 10Hz 周期帧，**没有** admin 管理 API（日志/参数/carinfo）。
「当前值」类需求一律走 `/state`，不靠推流。

- 服务代码：`openpilot/mate/server.py`（事件推导在 `openpilot/mate/poller.py`）
- 默认端口：**8083**（env `MATE_PORT` 可覆盖）
- 数据格式：JSON（UTF-8）
- 进程注册：manager `always_run`，开机常驻（offroad 也可连）

---

## 1. 接入

### 1.1 端点

| 端点 | 协议 | 鉴权 | 说明 |
|---|---|---|---|
| `ws://<车机IP>:8083/ws` | WebSocket | ✔ | 双向：收事件，可发 subscribe / snapshot / ping |
| `http://<车机IP>:8083/stream` | SSE | ✔ | 单向推流，`?kinds=events,sound` 过滤 |
| `http://<车机IP>:8083/state` | HTTP GET | ✔ | 一次性全量状态快照（见 §5） |
| `http://<车机IP>:8083/sounds/<file.wav>` | HTTP GET | ✔ | sound 事件对应音效文件 |
| `http://<车机IP>:8083/health` | HTTP GET | ✘ | 服务健康状态 |

车机 IP 在车机设置 / 路由器管理页查看，客户端需与车机在同一局域网。

### 1.2 鉴权（可选）

默认不校验。写入 token 后，除 `/health` 外所有接口必须携带：

```bash
echo -n "your-secret" > /data/params/d/MateToken    # 开
rm /data/params/d/MateToken                          # 关
```

改完约 1 秒内生效（服务端缓存），无需重启进程。客户端三种方式任选：

- Query 参数：`ws://host:8083/ws?token=your-secret`
- 请求头：`Authorization: Bearer your-secret`
- SSE 用 query 参数（浏览器 EventSource 不支持自定义头）

鉴权失败返回 HTTP 401 `{"error":"invalid token"}`。

### 1.3 事件类型（kind）

| kind | 触发时机 | 进 /state 快照 | 新连接补发 |
|---|---|---|---|
| `events` | onroad 告警集合**变化**（去抖） | ✔ | ✔（有活跃告警时） |
| `opstate` | openpilot 状态机跳转 / enabled/active/engageable 变化 | ✔ | ✘ |
| `button` | 方向盘按键按下/松开（边沿帧） | ✘ | ✘ |
| `sound` | 播报声音切换（含 carrot 倒计时报数） | ✔ | ✔（正在播报时） |

---

## 2. WebSocket 协议

### 2.1 服务端 → 客户端

所有消息统一信封：

```json
{ "type": "<类型>", "data": { ... } }
```

| type | 触发时机 | 说明 |
|---|---|---|
| `hello` | 连接建立后第一条 | `{"service":"mate","version":1,"port":8083,"kinds":[...]}` |
| `events` / `opstate` / `button` / `sound` | 见 §4 | 事件本体 |
| `snapshot` | 响应 snapshot 请求 | 见 §5 |
| `subscribed` | 响应 subscribe 请求 | `{"kinds":[...]}` 当前生效过滤 |
| `pong` | 响应 ping | `{"ts": <monotonic>}` |

连接建立后依次收到：`hello` → （当前活跃告警的 `events` 补发，有则发）→
（当前正在播报的 `sound` 补发，有则发）。**按键不补发**——客户端拿当前车辆状态请用
`snapshot` / `GET /state`。

服务端每 20s 发 WebSocket protocol ping，死连接自动清理。

### 2.2 客户端 → 服务端

```jsonc
// 订阅过滤（不发送则默认全部 4 类；重复发送覆盖之前的选择）
{ "type": "subscribe", "kinds": ["events", "sound"] }

// 请求一次全量状态快照（立即返回，不影响后续推送）
{ "type": "snapshot" }

// 心跳 / 测延迟
{ "type": "ping" }
```

未知消息与非法 JSON 直接忽略（不报错），便于客户端版本迭代。

### 2.3 JavaScript 示例

```javascript
const ws = new WebSocket("ws://192.168.1.100:8083/ws?token=your-secret");

ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);
  switch (msg.type) {
    case "hello":
      // 初始化：拿一份快照补齐当前状态
      ws.send(JSON.stringify({ type: "snapshot" }));
      break;
    case "snapshot":
      console.log("当前 OP 状态:", msg.data.data.opstate?.data.state);
      console.log("车速 km/h:", msg.data.data.car?.data.vEgoKph);
      break;
    case "events":
      for (const n of msg.data.added) console.log("新告警:", n);
      for (const n of msg.data.removed) console.log("告警解除:", n);
      break;
    case "opstate":
      console.log("OP:", msg.data.state, "active =", msg.data.active);
      break;
    case "button":
      console.log("按键:", msg.data.button, msg.data.pressed ? "按下" : "松开");
      break;
    case "sound":
      console.log("播报:", msg.data.sound, msg.data.loop ? "循环" : "单次");
      break;
  }
};
```

### 2.4 Python 示例（websocket-client，仓库已依赖）

```python
import json
import websocket

def on_message(ws, message):
    msg = json.loads(message)
    if msg["type"] in ("events", "opstate", "button", "sound"):
        print(msg["type"], msg["data"])

ws = websocket.WebSocketApp("ws://192.168.1.100:8083/ws?token=your-secret",
                            on_message=on_message)
ws.run_forever()
```

---

## 3. SSE 协议

- 地址：`http://<车机IP>:8083/stream?kinds=events,sound&token=your-secret`
  （`kinds` 省略 = 全部；非法 kind 被忽略）
- SSE 的 `event:` 名即消息类型（`events`/`opstate`/`button`/`sound`），
  `data:` 为同样的 JSON 信封
- 新连接同样先补发活跃告警与正在播报的声音
- 空闲 15s 输出一条 `: ping` 注释行保活
- 慢客户端：队列上限 256，满了丢最旧（事件频率很低，正常使用不会触发）
- 浏览器 `EventSource` 断线自动重连

```bash
curl -N "http://<车机IP>:8083/stream?kinds=events,opstate"
```

---

## 4. 事件数据字典

### 4.1 `events` 告警事件

onroad 告警集合**发生变化时**推送（出现/消失去抖，不每帧重复）：

```json
{
  "type": "events",
  "data": {
    "added":   ["doorOpen", "seatbeltNotLatched"],
    "removed": ["belowSteerSpeed"],
    "active":  [ { "name": "doorOpen", "warning": true, "...": "..." } ]
  }
}
```

`active` 每项字段（与本仓 `cereal/log.capnp` `OnroadEvent` 一致）：

| 字段 | 类型 | 含义 |
|---|---|---|
| `name` | string | 事件枚举名（完整列表见 `OnroadEvent.EventName`） |
| `warning` | bool | 黄/红警告（enabled 时展示） |
| `userDisable` | bool | 用户操作导致退出（踏板/转向/按键接管） |
| `softDisable` | bool | 软退出（可自动恢复） |
| `immediateDisable` | bool | 立即退出 |
| `permanent` | bool | 常驻告警（无论 OP 状态都展示） |
| `noEntry` | bool | 阻止 engage（解释了"为什么按了没反应"） |
| `preEnable` | bool | pre-enable 阶段提示 |
| `enable` | bool | 事件本身使能 |
| `overrideLateral` | bool | 横向被接管 |
| `overrideLongitudinal` | bool | 纵向被接管 |

> 粒度说明：事件集合变化即推，**同一事件仅 flag 变化不会重推**；客户端要拿
> 最新 flag 以 `active` 全量为准。BYD 上 `onroadEvents` 由 selfdrived 发布，
> 边沿粒度约一个发布周期（100ms 级）。

### 4.2 `opstate` 状态机跳转

`state` / `enabled` / `active` / `engageable` 任一变化时推送：

```json
{ "type": "opstate", "data": { "state": "enabled", "enabled": true, "active": true, "engageable": false } }
```

| state | 含义 |
|---|---|
| `disabled` | 未开启 |
| `preEnabled` | 预开启（踩刹车待命中） |
| `enabled` | 已开启（在控车，`active=true`） |
| `softDisabling` | 软退出中（可自动恢复） |
| `overriding` | 驾驶员接管中（油门/转向） |

### 4.3 `button` 方向盘按键

边沿事件，按下/松开各一条；长按不重复发：

```json
{ "type": "button", "data": { "button": "setCruise", "pressed": true } }
```

| `button` 值 | 含义 | | `button` 值 | 含义 |
|---|---|---|---|---|
| `mainCruise` | 巡航总开关 | | `gapAdjustCruise` | 跟车时距 |
| `setCruise` | SET− 设定 | | `lkas` | LKAS 按钮 |
| `resumeCruise` | RES+ 恢复 | | `lfaButton` | LFA 按钮（部分车型） |
| `accelCruise` | 巡航加速 | | `leftBlinker` / `rightBlinker` | 转向拨杆 |
| `decelCruise` | 巡航减速 | | `paddleLeft` / `paddleRight` | 换挡拨片 |
| `cancel` | 取消巡航 | | `altButton2` / `unknown` | 备用 / 未识别 |

> 转向灯持续状态请用 `/state` 的 `car.leftBlinker/rightBlinker`；
> 此事件是拨杆的**瞬间动作**。BYD 宋PLUS 注意：下拨 SET 厂商即激活（SET 是激活语义
> 而非设定，属厂商行为），客户端别把 `setCruise` 按下直接映射成"设定巡航"文案。

### 4.4 `sound` 播报声音

与本机 `soundd` 的发声判定完全同链（含 carrot 倒计时报数、巡航主开关长按提示）：

```json
{
  "type": "sound",
  "data": {
    "sound": "stopStop",
    "file": "audio_stopstop.wav",
    "loop": true,
    "alertText1": "BRAKE!",
    "alertText2": "Risk of Collision"
  }
}
```

| 字段 | 说明 |
|---|---|
| `sound` | `AudibleAlert` 枚举名；`none` 表示播报结束 |
| `file` | wav 文件名，可 `GET /sounds/<file>` 拉取；`none` 时为 `null` |
| `loop` | true=循环播到下一条 `sound`；false=播一遍 |
| `alertText1/2` | 车机屏幕上同步显示的告警文字（可能为空串） |

常见值（完整枚举见本仓 `opendbc/car/car.capnp` `AudibleAlert`）：
`engage`/`disengage`/`refuse`/`prompt`/`promptRepeat`/`promptDistracted`/
`warningSoft`/`warningImmediate`/`stopStop`/`bsdWarning`/`stopping`/
`trafficSignGreen`/`trafficSignChanged`/`laneChange`/`audioTurn`/`reverseGear`/
`longEngaged`/`longDisengaged`/`autoHold`/`engage2`/`disengage2`/`speedDown`/
`trafficError`/`radarCutin`/`systemReady`/`audio1`…`audio10`（倒计时报数）。

实现细节：
- `file` 来自本仓 `soundd.py` 的 `sound_list`，随设备型号（tizi 的
  `engage_tizi.wav` 等）与语言设置（`SoundLanguageSetting`/`LanguageSetting` →
  `sounds`/`sounds_chs`/`sounds_eng`）自动切换
- carrot 倒计时合成规则与 soundd 一致：`leftSec` 10→1 报数、0 → `longDisengaged`、
  11 → `promptDistracted`
- **与 soundd 的唯一差异**：soundd 会等有限长音效播完才转 `none`，mate 没有音频时钟，
  推导结束即推 `none`（客户端只会提前停播，不会漏停）

---

## 5. `GET /state` 与 WS `snapshot`

一次性返回当前全量状态快照（两者数据体相同）：

```json
{
  "ts": 1791472951.467,
  "data": {
    "car":     { "ts": 1791472951.400, "data": { "vEgoKph": 54.0, "...": "..." } },
    "opstate": { "ts": 1791472940.123, "data": { "state": "enabled", "...": "..." } },
    "events":  { "ts": 1791472947.925, "data": { "added": [...], "removed": [...], "active": [...] } },
    "sound":   { "ts": 1791472948.300, "data": { "sound": "stopStop", "...": "..." } }
  }
}
```

- 每段自带 `ts`（Unix 墙钟秒，车机时钟错时自行对齐），客户端据此判断新鲜度
- **未收到过的段不出现**（offroad 刚启动时 `car`/`opstate` 可能缺失）；
  `button` 永远不在快照里（瞬时边沿）
- `events` 段在告警清空后保留空 `active`（表示"确认无告警"，区别于未收到）

### `car` 段字段

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `vEgo` / `vEgoKph` | float | m/s, km/h | 最佳估计车速 |
| `vEgoCluster` | float | m/s | 仪表显示车速 |
| `aEgo` | float | m/s² | 估计加速度 |
| `standstill` | bool | | 静止 |
| `gearShifter` | string | | `unknown`/`park`/`reverse`/`neutral`/`drive`/`sport`/`low`/`brake`/`eco`/`manumatic` |
| `gas` / `brake` | float | 0–1 | 踏板开度（仅驾驶员） |
| `gasPressed` / `brakePressed` | bool | | 踏板踩下 |
| `steeringAngleDeg` | float | deg | 方向盘转角 |
| `steeringPressed` | bool | | 手在方向盘上 |
| `cruiseState` | object | | `{enabled, available, speed, speedKph, speedCluster, standstill, nonAdaptive}` |
| `vCruise` / `vCruiseKph` | float | m/s, km/h | 巡航设定速度 |
| `leftBlinker` / `rightBlinker` | bool | | 转向灯 |
| `doorOpen` | bool | | 车门开启 |
| `seatbeltUnlatched` | bool | | 安全带未系 |
| `parkingBrake` | bool | | 手刹 |
| `brakeHoldActive` | bool | | 自动驻车 |
| `latEnabled` | bool | | 横向控制激活 |
| `speedLimit` / `speedLimitKph` | float | m/s, km/h | 限速 |
| `engineRpm` | float | rpm | 发动机转速 |
| `accFaulted` / `espDisabled` | bool | | ACC 故障 / ESP 关闭 |
| `canValid` / `canTimeout` | bool | | CAN 校验 / 掉线 |
| `carrotCruise` | int | | carrot 巡航设定 |

> 快照在 card 停发（offroad）后保持最后一次 onroad 值，新鲜度看 `car.ts`。

---

## 6. 其余接口

### `GET /sounds/<file.wav>`

sound 事件的配套音效。目录随语言设置切换（`sounds` / `sounds_chs` / `sounds_eng`，
缺失文件回退英文目录），仅接受单个 `.wav` 文件名（路径穿越一律 404）。
浏览器可直接 `<audio src="http://<车机IP>:8083/sounds/engage.wav">`。

### `GET /health`

```json
{ "ok": true, "service": "mate", "uptime": 1234.5, "clients": 2, "token_auth": false }
```

不鉴权，供探活脚本使用。

---

## 7. 客户端开发建议

- **初始化流程**：连 WS → 收 `hello` → 主动发 `{"type":"snapshot"}`（或 HTTP GET
  `/state`）建立当前状态 → 之后纯靠事件维护增量。重连后重复此流程（补发的
  `events`/`sound` 已覆盖告警与播报，但车速/档位要另取快照）。
- **重连**：指数退避；SSE 浏览器自带。
- **对时**：用每段 `ts`（车机墙钟），不要用客户端本地收包时间推算状态时序。
- **枚举容错**：所有枚举为字符串，未来新增值可能显示为 `"unknown"`，客户端要兜底。
- **流量**：事件为低频边沿（全订阅约 1–3 条/秒，峰值出现在按键/告警瞬间），
  无需 topic 级节流；`/state` 一次约 2–4 KB。
- **不要公网暴露**：设计为局域网使用；远程访问走 VPN/内网穿透并务必设置 `MateToken`。

## 8. 部署

本仓库已完成注册（`openpilot/system/manager/process_config.py`）：

```python
PythonProcess("mate", "openpilot.mate.server", always_run, restart_if_crash=True),
```

车机部署：把 `openpilot/mate/` 整个目录放到 `/data/openpilot/openpilot/mate/`，
连同修改后的 `process_config.py`，重启 manager 生效。手动调试（前台，Ctrl+C 停）：

```bash
cd /data/openpilot && source launch_env.sh && python -m openpilot.mate
```

验证：`curl http://localhost:8083/health` 返回 `"ok": true`；
`curl -N http://localhost:8083/stream` 能看到 `hello` 后的补发与边沿事件。
卸载：删注册行重启 manager，删 `openpilot/mate/` 目录与
`/data/params/d/MateToken` 即可，无其他残留。

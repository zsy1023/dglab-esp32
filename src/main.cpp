/*
 * dglab-esp32 —— 在 ESP32 上复刻 DG-LAB 郊狼主机（Coyote）3.0 的 BLE 外设。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 zsyfpv
 *
 * 本文件是 dglab-esp32 的一部分，以 GNU GPL v3.0（或更新版本）发布，详见随附的 LICENSE。
 *
 * 职责边界（**板子自己就是一台主机**）：
 *   ✅ 复刻真机 GATT 全表 + 广播名 47L121000 + App 订阅 0x150B 后的固定握手
 *   ✅ ★ 主机逻辑：解析 App 的 B0 / BF → 维护 A/B 通道强度、软上限、波形 → 需要时回 B1
 *        （真机不需要 PC，本固件也不需要：手机单独连板子就能调强度、能看到强度回显）
 *   ✅ 原始 B0/BF 照样经串口上行给 PC（PC 侧从此只当"观察者"，**不要再回 B1**）
 *        默认不发：只有当 PC 发来 0xCE 心跳（上位机启动时的在线探测）后才开始发，
 *        这样"手机单独连板子"时串口监视器里只有干净的文本日志
 *   ✅ 串口日志（**纯 ASCII**）把强度 / 波形 / 软上限直接打出来，串口监视器就能看
 *   ❌ 不做叠加 / 惩罚 / 规则 / 网络 —— 那些仍然是 PC 侧上位机的事
 *
 * 权威依据：官方逐字节规格 reference/dglab-bluetooth-protocol/coyote/v3/README.md
 *          + 真机实测 docs/research/郊狼真机GATT实测.md（实测优先于文档）
 * 接口契约：本文件下方的「串口帧协议」常量 + ../README.md §4
 *
 * 已实测的关键点（照做，别照官方文档改）：
 *   1. 0x150A 是 WriteWithoutResponse（不是普通 WRITE）；
 *   2. 0x180A/0x1501 必须返回 "0c 03"，App 靠它判定协议版本 V3；
 *   3. App 初始化流程：read(180A/1500) → read(180A/1501) → notify(180A/1500)
 *      → notify(180C/150B) → check(180C/150A) → notify(180A/2A59)。
 *
 * ★ 日志一律纯 ASCII 英文：早期版本打的是 UTF-8 中文，在按 GBK 解码的串口监视器里
 *   会变成「宸茶繛鎺ワ（conn_id=0锛」这种乱码。ASCII 在任何编码下都不会乱。
 */

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <esp_system.h>
#include <stdarg.h>

#include "host_state.h"  // 主机逻辑（纯 C++，无 Arduino 依赖 → 可脱硬件单测）

// ══════════════════════════════════════════════════════════════
// 串口帧协议（与 PC 侧上位机一致；契约见 ../README.md §4）
//   上行（ESP32 → PC）  0xB0+19B(B0 原样) | 0xBF+6B(BF 原样) | 0xCC(已连) | 0xCD(已断)
//   下行（PC → ESP32）  0xCE        心跳 / 在线探测（板子回一条 '#' 日志，并顺手打开原始帧上行）
//                      0xC0/0xC1  关 / 开「原始帧上行」（默认关；收到 0xCE 自动开，调试时可手动切）
//                      0xC2/0xC3  关 / 开「逐条 B0 详单」（默认关）
//                      0xB1+3B    留作兼容 —— 板子自己做主机后**不再采用**（见 handleDownlinkFrame）
//   靠 HEAD 字节定长分帧，没有长度前缀、没有校验
// ══════════════════════════════════════════════════════════════
static const uint8_t UP_B0 = 0xB0;
static const uint8_t UP_BF = 0xBF;
static const uint8_t UP_CONNECTED = 0xCC;
static const uint8_t UP_DISCONNECTED = 0xCD;
static const uint8_t DOWN_B1 = 0xB1;
static const uint8_t DOWN_HEARTBEAT = 0xCE;
static const uint8_t DOWN_RAW_OFF = 0xC0;
static const uint8_t DOWN_RAW_ON = 0xC1;
static const uint8_t DOWN_VERBOSE_OFF = 0xC2;
static const uint8_t DOWN_VERBOSE_ON = 0xC3;
//: 链路指令（给网页/上位机当"拨轮 + 回路开关"用）—— 都是「HEAD + 通道 + 参数」3 字节
//:   0xD0 ch value   强度绝对值（0~200，超过 200 按 200）
//:   0xD1 ch delta   强度相对增减（int8，负数 = 减）
//:   0xD2 ch on      回路开关（0 = 断开 / 非 0 = 闭合）
//:   ch: 0 = A 通道，1 = B 通道，0xFF = 两个都改
static const uint8_t DOWN_STRENGTH_SET = 0xD0;
static const uint8_t DOWN_STRENGTH_STEP = 0xD1;
static const uint8_t DOWN_LOOP = 0xD2;
static const uint8_t CH_BOTH = 0xFF;
static const size_t LEN_DOWN_CMD = 3;  // HEAD + ch + arg

static const size_t LEN_B0 = 20;  // 0xB0 + 19B
static const size_t LEN_BF = 7;   // 0xBF + 6B
static const size_t LEN_B1 = 4;   // 0xB1 + 3B

// 真机广播名 / 设备名（0x1800 / 0x2A00）
static const char *DEVICE_NAME = "47L121000";
// 序列号前缀（真机 0x180A/0x2A25 读回 "CEB3%Q" + 本机 MAC）
static const char *SERIAL_PREFIX = "CEB3%Q";

// ══════════════════════════════════════════════════════════════
// 主机逻辑：状态机在 host_state.h（纯逻辑，可脱硬件单测），这里只做"接线 + 打日志"
// ══════════════════════════════════════════════════════════════
static dglab::Host g_host;
//: 常用常量的本地别名（日志/夹值代码里读起来短一点）
static const uint8_t STRENGTH_MAX = dglab::STRENGTH_MAX;


// ══════════════════════════════════════════════════════════════
// 受控侧 / 链路状态
// ══════════════════════════════════════════════════════════════
static uint8_t g_mac[6] = {0, 0, 0, 0, 0, 0};

static BLEServer *g_server = nullptr;
static BLECharacteristic *g_chr_notify = nullptr;   // 0x180C / 0x150B
static BLECharacteristic *g_chr_battery = nullptr;  // 0x180A / 0x1500
static BLECharacteristic *g_chr_status = nullptr;   // 0x180A / 0x2A59
static BLE2902 *g_cccd_notify = nullptr;
static BLE2902 *g_cccd_battery = nullptr;
static BLE2902 *g_cccd_status = nullptr;
static BLE2902 *g_cccd_0008 = nullptr;

//: 0x2A59（10 字节，输出状态回读）。基线取自真机实测；
//: 字节 2/3 ↔ A/B 通道（近似复刻：跟着 B0 里的波形强度走），精确编码仍未解出。
static uint8_t g_status[10] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x4a, 0x4a, 0x4a};
static volatile bool g_status_dirty = false;

static volatile bool g_app_connected = false;
static bool g_prev_subscribed = false;
static unsigned long g_last_battery_ms = 0;
static bool g_restart_adv = false;
static unsigned long g_restart_adv_at = 0;
//: 是否已经为当前这次连接请求过快速连接参数（避免反复请求）
static bool g_conn_param_requested = false;

//: 运行期开关（串口可以随时切换，见 §串口帧协议）
//: 原始帧上行默认**关**：手机单独连板子时串口监视器就只有干净的 '#' 文本行；
//: 一旦收到 PC 的 0xCE 心跳（上位机启动时的在线探测），自动打开 —— 上位机侧代码不用改。
static volatile bool g_raw_uplink = false;  // 原始 B0/BF 是否上行给 PC
//: 逐条 B0 详单**默认开**：App 每 100ms 写一条 B0，每通道 4 组波形、每组 25ms ——
//: 这就是协议能给到的最快刷新率。想安静就发 0xC2 关掉。
static volatile bool g_verbose_b0 = true;

//: 诊断统计（排障一眼看出：指令进来多少、B1 回了几条、下行有没有字节）
static uint32_t g_stat_b0 = 0;        // 收到的 B0 条数
static uint32_t g_stat_bf = 0;        // 收到的 BF 条数
static uint32_t g_stat_b1 = 0;        // 发给 App 的 B1 条数
static uint32_t g_stat_drop = 0;      // 非主机指令 / 长度不符、被丢弃的写入
static uint32_t g_stat_down_bytes = 0;    // 从 PC 收到的下行字节数
static uint32_t g_stat_down_b1 = 0;       // 下行里被忽略的 B1（PC 侧老逻辑还在发）
static uint32_t g_stat_down_unknown = 0;  // 下行里等待帧头时被丢弃的字节数
static uint32_t g_stat_log_drop = 0;      // 串口 TX 挤不下、被丢掉的日志行
static uint32_t g_b0_at_last_stat = 0;    // 上一轮 STAT 时的 B0 计数（算条/秒）
static unsigned long g_last_stat_ms = 0;

// ══════════════════════════════════════════════════════════════
// 日志：**统一 '#' 前缀 + 换行 + 整行一次写出**（纯 ASCII）
//   '#' 不在帧头集合里，PC 侧按行整行丢弃 → 日志和二进制帧共用同一个 UART 也不打架
// ══════════════════════════════════════════════════════════════
static void logLine(const char *fmt, ...) {
  char buf[320];
  size_t off = 0;
  buf[off++] = '#';
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf + off, sizeof(buf) - off - 1, fmt, ap);
  va_end(ap);
  if (n > 0) {
    off += (size_t)n;
    if (off > sizeof(buf) - 2) {
      off = sizeof(buf) - 2;
    }
  }
  buf[off++] = '\n';
  // 一次 write 写完一整行：BLE 任务和 loop 任务都在打日志，别让两行交错成半行。
  // ★ 攒不下就**丢这一行**（只记个数），绝不在 BLE 回调里阻塞 —— 日志不能拖住协议栈
  if ((size_t)Serial.availableForWrite() < off) {
    ++g_stat_log_drop;
    return;
  }
  Serial.write((const uint8_t *)buf, off);
}

//: 把一串字节打成 hex 附在日志里（最多 24 字节）—— 排障时"App 到底写了什么"最有用
static void logHexBytes(const char *tag, const uint8_t *p, size_t len) {
  char hex[3 * 24 + 1];
  size_t n = (len < 24) ? len : 24;
  size_t off = 0;
  for (size_t i = 0; i < n && off + 4 < sizeof(hex); ++i) {
    off += (size_t)snprintf(hex + off, sizeof(hex) - off, "%02X ", p[i]);
  }
  if (off > 0) {
    hex[off - 1] = '\0';  // 去掉末尾空格
  } else {
    hex[0] = '\0';
  }
  logLine("%s len=%u %s%s", tag, (unsigned)len, hex, (len > n) ? " ..." : "");
}

//: 把一行原始 B0 打成 hex（"为什么没波形"的最终证据）
static void formatHex(char *out, size_t cap, const uint8_t *p, size_t len) {
  size_t off = 0;
  for (size_t i = 0; i < len && off + 3 < cap; ++i) {
    int m = snprintf(out + off, cap - off, "%02X", p[i]);
    if (m <= 0) {
      break;
    }
    off += (size_t)m;
  }
}

//: 4 组波形压成 "freq:amp freq:amp ..."；整组作废时打 "!" 前缀（实现在 host_state.h）
static void formatWave(char *out, size_t cap, const dglab::ChannelWave &w) {
  dglab::Host::formatWave(out, cap, w);
}

static void sendUplink(const uint8_t *data, size_t len) {
  Serial.write(data, len);
}

//: 原始帧上行（PC 侧当观察者用）；串口发 0xC0 可以关掉，让监视器只剩 '#' 文本行
static void forwardUplink(const uint8_t *data, size_t len) {
  if (!g_raw_uplink) {
    return;
  }
  Serial.write(data, len);
}

// ══════════════════════════════════════════════════════════════
// 软上限 / 平衡参数：真机断电保存 —— 存 NVS（只在值变化时写，省 flash 寿命）
// ══════════════════════════════════════════════════════════════
static Preferences g_prefs;

static void loadPersistedSettings() {
  g_prefs.begin("dglab", false);
  uint8_t cap_a = g_prefs.getUChar("capA", STRENGTH_MAX);
  uint8_t cap_b = g_prefs.getUChar("capB", STRENGTH_MAX);
  g_host.setSoftCaps(cap_a, cap_b);
}

static void saveSoftCaps() {
  g_prefs.putUChar("capA", g_host.softCap(0));
  g_prefs.putUChar("capB", g_host.softCap(1));
}

// ══════════════════════════════════════════════════════════════
// 主机逻辑：B0 / BF → 状态推进 → B1
// ══════════════════════════════════════════════════════════════
//: B1 队列（写入回调在 BLE 任务里，notify 放到 loop 里做 —— 不在 BLE 回调里重入协议栈）
static const int B1_QUEUE = 4;
static uint8_t g_b1_q[B1_QUEUE][LEN_B1];
static int g_b1_head = 0;  // 写指针（BLE 任务）
static int g_b1_tail = 0;  // 读指针（loop 任务）
static portMUX_TYPE g_b1_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_b1_overflow = 0;

static void pushB1(uint8_t seq, uint8_t a, uint8_t b) {
  portENTER_CRITICAL(&g_b1_mux);
  int next = (g_b1_head + 1) % B1_QUEUE;
  if (next == g_b1_tail) {  // 满了：丢最老的（最新强度最重要）
    g_b1_tail = (g_b1_tail + 1) % B1_QUEUE;
    ++g_b1_overflow;
  }
  g_b1_q[g_b1_head][0] = DOWN_B1;
  g_b1_q[g_b1_head][1] = seq;
  g_b1_q[g_b1_head][2] = a;
  g_b1_q[g_b1_head][3] = b;
  g_b1_head = next;
  portEXIT_CRITICAL(&g_b1_mux);
}

static bool popB1(uint8_t *out) {
  bool ok = false;
  portENTER_CRITICAL(&g_b1_mux);
  if (g_b1_tail != g_b1_head) {
    memcpy(out, g_b1_q[g_b1_tail], LEN_B1);
    g_b1_tail = (g_b1_tail + 1) % B1_QUEUE;
    ok = true;
  }
  portEXIT_CRITICAL(&g_b1_mux);
  return ok;
}

//: 应用一条 B0：主机状态推进（强度 + 波形）→ 该回 B1 就排进队列
static void applyB0(const uint8_t *p, size_t len) {
  dglab::B0Result r = g_host.applyB0(p, len);
  if (!r.ok) {
    return;
  }

  // 0x2A59 近似复刻：字节 2/3 跟 A/B 通道首个波形强度走
  uint8_t amp_a = g_host.wave(0).valid ? g_host.wave(0).amp[0] : 0;
  uint8_t amp_b = g_host.wave(1).valid ? g_host.wave(1).amp[0] : 0;
  if (g_status[2] != amp_a || g_status[3] != amp_b) {
    g_status[2] = amp_a;
    g_status[3] = amp_b;
    g_status_dirty = true;
  }

  if (g_verbose_b0) {
    char wave_a[64];
    char wave_b[64];
    char raw[2 * LEN_B0 + 1];
    dglab::Host::formatWave(wave_a, sizeof(wave_a), g_host.wave(0));
    dglab::Host::formatWave(wave_b, sizeof(wave_b), g_host.wave(1));
    formatHex(raw, sizeof(raw), p, len);
    // ★ 每条 B0 都打（App 每 100ms 一条，每通道 4 组 × 25ms = 协议能给的最快刷新率）
    //   link=A1/B1 是回路状态（1 = 闭合 / 0 = 断开）
    //   wA/wB 前面带 `!` = 该通道整组作废（真机不输出）；raw = 这条 B0 的原始字节，排障的最终证据
    logLine("B0 t=%lu seq=%u mode=A%c/B%c set=%u/%u A=%u/%u B=%u/%u link=A%d/B%d wA=%s wB=%s raw=%s",
            millis(), (unsigned)r.seq, dglab::Host::modeChar(r.mode_a),
            dglab::Host::modeChar(r.mode_b), (unsigned)r.setpoint_a, (unsigned)r.setpoint_b,
            (unsigned)g_host.strength(0), (unsigned)g_host.softCap(0), (unsigned)g_host.strength(1),
            (unsigned)g_host.softCap(1), g_host.loopEnabled(0) ? 1 : 0,
            g_host.loopEnabled(1) ? 1 : 0, wave_a, wave_b, raw);
  }

  if (r.changed_a || r.changed_b) {
    logLine("STRENGTH A=%u/%u B=%u/%u (seq=%u)", (unsigned)g_host.strength(0),
            (unsigned)g_host.softCap(0), (unsigned)g_host.strength(1), (unsigned)g_host.softCap(1),
            (unsigned)r.seq);
  }

  // 真机实测：序列号 ≠ 0 就必回一条同序列号的 B1（哪怕强度一个字节没变）
  if (r.wants_reply()) {
    pushB1(r.seq, g_host.strength(0), g_host.strength(1));
  }
}

//: 应用一条 BF：软上限（>200 不修改）+ 平衡参数；收紧软上限会立刻把强度压回去
static void applyBf(const uint8_t *p, size_t len) {
  dglab::BfResult r = g_host.applyBf(p, len);
  if (!r.ok) {
    return;
  }
  if (r.cap_changed) {
    saveSoftCaps();  // 真机断电保存
  }
  logLine("SOFT LIMIT A=%u B=%u | freq_bal A=%u B=%u | amp_bal A=%u B=%u%s",
          (unsigned)g_host.softCap(0), (unsigned)g_host.softCap(1),
          (unsigned)g_host.freqBalance(0), (unsigned)g_host.freqBalance(1),
          (unsigned)g_host.ampBalance(0), (unsigned)g_host.ampBalance(1),
          r.cap_changed ? " (saved to flash)" : "");
  if (r.clamped) {
    logLine("STRENGTH A=%u/%u B=%u/%u (clamped by new soft cap)", (unsigned)g_host.strength(0),
            (unsigned)g_host.softCap(0), (unsigned)g_host.strength(1), (unsigned)g_host.softCap(1));
  }
}

static void notifyStatusIfDirty() {
  if (!g_status_dirty || g_chr_status == nullptr) {
    return;
  }
  g_status_dirty = false;
  g_chr_status->setValue(g_status, sizeof(g_status));
  if (g_cccd_status != nullptr && g_cccd_status->getNotifications()) {
    g_chr_status->notify();
  }
}

//: 把排队的 B1 逐条 notify 给 App（在 loop 任务里做）
static void flushB1Queue() {
  uint8_t frame[LEN_B1];
  while (popB1(frame)) {
    if (g_chr_notify == nullptr) {
      return;
    }
    g_chr_notify->setValue(frame, LEN_B1);
    if (g_cccd_notify != nullptr && g_cccd_notify->getNotifications()) {
      g_chr_notify->notify();
      ++g_stat_b1;
      logLine("B1 -> App seq=%u A=%u B=%u", (unsigned)frame[1], (unsigned)frame[2],
              (unsigned)frame[3]);
    } else {
      logLine("B1 dropped seq=%u: 0x150B not subscribed", (unsigned)frame[1]);
    }
  }
}

// ══════════════════════════════════════════════════════════════
// GAP 回调（**额外订阅** —— Arduino 的 gapEventHandler 会先调它、再走自己的默认处理）
// 用途：① 把实际生效的 BLE 连接间隔打出来（判断 App 到底用了多快的心跳）；
//       ② 拿到对端地址后请求一次更快的连接参数（App 是 central，最终仍由它决定）
// ══════════════════════════════════════════════════════════════
static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event != ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT) {
    return;
  }
  const auto &up = param->update_conn_params;
  logLine("conn params: interval=%u (%.1f ms) latency=%u timeout=%u status=%d",
          (unsigned)up.conn_int, up.conn_int * 1.25f, (unsigned)up.latency, (unsigned)up.timeout,
          (int)up.status);
  if (g_conn_param_requested) {
    return;
  }
  g_conn_param_requested = true;
  esp_ble_conn_update_params_t p = {};
  memcpy(p.bda, up.bda, sizeof(p.bda));
  p.min_int = 12;   // 15 ms（单位 1.25ms）
  p.max_int = 24;   // 30 ms
  p.latency = 0;    // 不做 slave latency（回包更及时）
  p.timeout = 200;  // 2 s（单位 10ms）
  esp_err_t rc = esp_ble_gap_update_conn_params(&p);
  logLine("request fast conn params (15~30ms, no latency): rc=%d", (int)rc);
}

// ══════════════════════════════════════════════════════════════
// BLE 回调
// ══════════════════════════════════════════════════════════════
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    g_app_connected = true;
    g_conn_param_requested = false;  // 由 GAP 回调在拿到对端地址后请求一次快速连接参数
    uint8_t evt = UP_CONNECTED;
    sendUplink(&evt, 1);
    logLine("App connected (conn_id=%d) | strength A=%u/%u B=%u/%u", server->getConnId(),
            (unsigned)g_host.strength(0), (unsigned)g_host.softCap(0), (unsigned)g_host.strength(1),
            (unsigned)g_host.softCap(1));
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    g_app_connected = false;
    g_prev_subscribed = false;
    uint8_t evt = UP_DISCONNECTED;
    sendUplink(&evt, 1);
    logLine("App disconnected -> restart advertising");
    // 断开后必须自己重启广播，否则手机再也搜不到（验收 A9）
    g_restart_adv = true;
    g_restart_adv_at = millis() + 300;
  }
};

//: 0x180C/0x150A（App 的所有指令）：主机逻辑 + 原始帧上行 + 记录
class CoyoteWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    const uint8_t *p = chr->getData();
    size_t len = chr->getLength();
    if (p == nullptr || len == 0) {
      return;
    }
    if (len == LEN_B0 && p[0] == UP_B0) {
      ++g_stat_b0;
      applyB0(p, len);  // ★ 板子自己就是主机：解析 + 维护状态 + 排 B1
      forwardUplink(p, len);
      return;
    }
    if (len == LEN_BF && p[0] == UP_BF) {
      ++g_stat_bf;
      applyBf(p, len);  // 软上限 / 平衡参数
      forwardUplink(p, len);
      return;
    }
    // 不按契约长发的写入不进链路（免得污染 PC 侧的定长分帧），但要**记下来**：
    // App 还会写灵猫/爪印那类「非主机指令」(0x50 / 0xBC / 0xC3 / 0xC4 / 0xFF …)，语义未知，忽略即可
    ++g_stat_drop;
    logHexBytes("IGNORED (not a host B0/BF opcode)", p, len);
  }
};

class UnknownWriteLogCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    const uint8_t *p = chr->getData();
    size_t len = chr->getLength();
    if (len == 0) {
      return;
    }
    logLine("unknown char 0x%s write %u bytes head=0x%02X", chr->getUUID().toString().c_str(),
            (unsigned)len, (unsigned)p[0]);
  }
};

// ══════════════════════════════════════════════════════════════
// GATT 表（复刻真机；0x1800 / 0x1801 由协议栈内建）
// ══════════════════════════════════════════════════════════════
static BLECharacteristic *addCharacteristic(BLEService *svc, uint16_t uuid, uint32_t props,
                                            BLE2902 **cccd = nullptr) {
  BLECharacteristic *chr = svc->createCharacteristic(BLEUUID(uuid), props);
  if (cccd != nullptr && (props & (BLECharacteristic::PROPERTY_NOTIFY |
                                   BLECharacteristic::PROPERTY_INDICATE)) != 0) {
    *cccd = new BLE2902();
    chr->addDescriptor(*cccd);
  }
  return chr;
}

static void buildGattTable() {
  // ── 0x180C：郊狼核心 ────────────────────────────────────────
  BLEService *svc180C = g_server->createService(BLEUUID((uint16_t)0x180C));
  BLECharacteristic *chr150A = addCharacteristic(svc180C, 0x150A,
                                                 BLECharacteristic::PROPERTY_WRITE_NR);
  chr150A->setCallbacks(new CoyoteWriteCallbacks());  // ★ WriteWithoutResponse，不是 WRITE
  g_chr_notify = addCharacteristic(svc180C, 0x150B, BLECharacteristic::PROPERTY_NOTIFY,
                                   &g_cccd_notify);
  svc180C->start();

  // ── 0x180A：设备信息（App 初始化要 read/notify 它们）────────
  BLEService *svc180A = g_server->createService(BLEUUID((uint16_t)0x180A));

  g_chr_battery = addCharacteristic(svc180A, 0x1500,
                                    BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_NOTIFY,
                                    &g_cccd_battery);
  uint8_t battery = 100;  // 真机读回 0x64 = 100%（1 字节百分比）
  g_chr_battery->setValue(&battery, 1);

  BLECharacteristic *chr1501 = addCharacteristic(svc180A, 0x1501,
                                                 BLECharacteristic::PROPERTY_READ);
  // ★ App 靠它判定协议版本：必须 0c 03（03 = V3）
  uint8_t ver[2] = {0x0C, 0x03};
  chr1501->setValue(ver, sizeof(ver));

  BLECharacteristic *chr1502 = addCharacteristic(svc180A, 0x1502,
                                                 BLECharacteristic::PROPERTY_READ);
  chr1502->setValue(g_mac, 6);  // 真机 = 本机 MAC（官方文档没这个特征）

  BLECharacteristic *chr2A25 = addCharacteristic(svc180A, 0x2A25,
                                                 BLECharacteristic::PROPERTY_READ);
  uint8_t serial[sizeof("CEB3%Q") - 1 + 6];
  memcpy(serial, SERIAL_PREFIX, sizeof("CEB3%Q") - 1);
  memcpy(serial + sizeof("CEB3%Q") - 1, g_mac, 6);
  chr2A25->setValue(serial, sizeof(serial));  // 前缀 + MAC

  g_chr_status = addCharacteristic(svc180A, 0x2A59,
                                   BLECharacteristic::PROPERTY_READ |
                                       BLECharacteristic::PROPERTY_NOTIFY,
                                   &g_cccd_status);
  g_chr_status->setValue(g_status, sizeof(g_status));  // 10 字节输出状态回读
  svc180A->start();

  // ── 0x2003 / 0x2004：真机有、用途未知（先提供更安全）──────
  BLEService *svc2003 = g_server->createService(BLEUUID((uint16_t)0x2003));
  BLECharacteristic *chr0007 = addCharacteristic(svc2003, 0x0007,
                                                 BLECharacteristic::PROPERTY_WRITE);
  chr0007->setCallbacks(new UnknownWriteLogCallbacks());
  addCharacteristic(svc2003, 0x0008,
                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY,
                    &g_cccd_0008);
  svc2003->start();

  BLEService *svc2004 = g_server->createService(BLEUUID((uint16_t)0x2004));
  BLECharacteristic *chr0009 = addCharacteristic(svc2004, 0x0009,
                                                 BLECharacteristic::PROPERTY_READ |
                                                     BLECharacteristic::PROPERTY_WRITE);
  uint8_t zero = 0x00;
  chr0009->setValue(&zero, 1);
  chr0009->setCallbacks(new UnknownWriteLogCallbacks());
  svc2004->start();
}

// ══════════════════════════════════════════════════════════════
// 下行（PC / 网页 → ESP32）
//   0xCE 心跳 → 回一条 '#' 日志（在线探测）
//   0xC0/0xC1 切「原始帧上行」，0xC2/0xC3 切「逐条 B0 详单」
//   0xD0/0xD1/0xD2 链路指令：强度绝对值 / 强度相对增减 / 回路开关（A、B 通道独立）
//   0xB1 → **忽略**：板子自己做主机后，强度回包以固件为准（老 PC 逻辑若还在发，会记一次警告）
// ══════════════════════════════════════════════════════════════
static uint8_t g_down_buf[LEN_B1];  // 最长的下行帧就是 B1（4 字节），3 字节的链路指令也放得下
static size_t g_down_got = 0;
static size_t g_down_need = 0;
static unsigned long g_down_last_ms = 0;
static const unsigned long DOWN_FRAME_TIMEOUT_MS = 200;

static bool isLinkCommand(uint8_t head) {
  return head == DOWN_STRENGTH_SET || head == DOWN_STRENGTH_STEP || head == DOWN_LOOP;
}

static void logLinkState() {
  logLine("link: loop A=%s B=%s | strength A=%u/%u B=%u/%u",
          g_host.loopEnabled(0) ? "closed" : "open", g_host.loopEnabled(1) ? "closed" : "open",
          (unsigned)g_host.strength(0), (unsigned)g_host.softCap(0), (unsigned)g_host.strength(1),
          (unsigned)g_host.softCap(1));
}

//: 网页/上位机发来的「拨轮 + 回路」指令：改状态 → 打日志 → 强度变了就回一条 B1(seq=0)
//: （真机拨轮改完强度也是回 seq=0 的 B1，App 靠它更新界面）
static void handleLinkCommand(uint8_t head, uint8_t ch, uint8_t arg) {
  if (ch != 0 && ch != 1 && ch != CH_BOTH) {
    logLine("link cmd 0x%02X: bad channel %u, dropped", (unsigned)head, (unsigned)ch);
    return;
  }
  const bool to_a = (ch == CH_BOTH || ch == 0);
  const bool to_b = (ch == CH_BOTH || ch == 1);

  if (head == DOWN_LOOP) {
    const bool on = (arg != 0);
    if (to_a) {
      g_host.setLoop(0, on);
    }
    if (to_b) {
      g_host.setLoop(1, on);
    }
    logLinkState();
    return;
  }

  bool changed = false;
  if (head == DOWN_STRENGTH_SET) {
    const uint8_t value = (arg > STRENGTH_MAX) ? STRENGTH_MAX : arg;  // 拨轮最大就是 200
    if (to_a) {
      changed |= g_host.manualStrength(0, dglab::MODE_SET, value);
    }
    if (to_b) {
      changed |= g_host.manualStrength(1, dglab::MODE_SET, value);
    }
  } else {  // DOWN_STRENGTH_STEP
    const int8_t delta = (int8_t)arg;
    const uint8_t mode = (delta < 0) ? dglab::MODE_SUB : dglab::MODE_ADD;
    const uint8_t mag = (uint8_t)((delta < 0) ? -delta : delta);
    if (to_a) {
      changed |= g_host.manualStrength(0, mode, mag);
    }
    if (to_b) {
      changed |= g_host.manualStrength(1, mode, mag);
    }
  }

  logLine("STRENGTH A=%u/%u B=%u/%u (via serial cmd 0x%02X)%s", (unsigned)g_host.strength(0),
          (unsigned)g_host.softCap(0), (unsigned)g_host.strength(1), (unsigned)g_host.softCap(1),
          (unsigned)head, changed ? "" : " (no change)");
  if (changed) {
    pushB1(0, g_host.strength(0), g_host.strength(1));
  }
}

static void handleDownlinkFrame() {
  const uint8_t head = g_down_buf[0];
  if (isLinkCommand(head)) {
    handleLinkCommand(head, g_down_buf[1], g_down_buf[2]);
    return;
  }
  if (head != DOWN_B1) {
    logLine("downlink bad head 0x%02X, dropped", (unsigned)head);
    return;
  }
  ++g_stat_down_b1;
  if (g_stat_down_b1 <= 3) {
    logLine("B1 from PC IGNORED (%02X %02X %02X %02X): firmware is the host now, PC must stop "
            "sending B1",
            (unsigned)g_down_buf[0], (unsigned)g_down_buf[1], (unsigned)g_down_buf[2],
            (unsigned)g_down_buf[3]);
  }
}

static void pollDownlink() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    ++g_stat_down_bytes;
    g_down_last_ms = millis();
    if (g_down_need == 0) {
      if (b == DOWN_B1 || isLinkCommand(b)) {
        // 帧头也算这一帧的一部分
        g_down_buf[0] = b;
        g_down_got = 1;
        g_down_need = (b == DOWN_B1) ? LEN_B1 : LEN_DOWN_CMD;
      } else if (b == DOWN_HEARTBEAT) {
        logLine("alive (heartbeat 0xCE)");
        if (!g_raw_uplink) {
          // 上位机在打招呼 → 说明 PC 侧在线，把原始帧上行打开（上位机老代码不用改任何东西）
          g_raw_uplink = true;
          logLine("raw uplink (B0/BF frames) = ON (PC heartbeat seen)");
        }
      } else if (b == DOWN_RAW_OFF || b == DOWN_RAW_ON) {
        g_raw_uplink = (b == DOWN_RAW_ON);
        logLine("raw uplink (B0/BF frames) = %s", g_raw_uplink ? "ON" : "OFF");
      } else if (b == DOWN_VERBOSE_OFF || b == DOWN_VERBOSE_ON) {
        g_verbose_b0 = (b == DOWN_VERBOSE_ON);
        logLine("verbose B0 dump = %s", g_verbose_b0 ? "ON" : "OFF");
      } else if (++g_stat_down_unknown <= 8) {
        // 只报前几次（开机时 ROM 日志的残渣会有一串，别刷屏）
        logLine("downlink unknown byte 0x%02X (waiting for head, dropped)", (unsigned)b);
      }
      continue;
    }
    g_down_buf[g_down_got++] = b;
    if (g_down_got >= g_down_need) {
      handleDownlinkFrame();
      g_down_need = 0;
      g_down_got = 0;
    }
  }
  // 半帧看门狗：收帧到一半就断了 → 丢弃，重新找帧头（否则一个丢字节会把下行永久卡死）
  if (g_down_need > 0 && (millis() - g_down_last_ms) > DOWN_FRAME_TIMEOUT_MS) {
    logLine("downlink half frame timeout (%u/%u bytes) -> resync", (unsigned)g_down_got,
            (unsigned)g_down_need);
    g_down_need = 0;
    g_down_got = 0;
  }
}

// ══════════════════════════════════════════════════════════════
// 周期任务
// ══════════════════════════════════════════════════════════════
//: 真机行为：**订阅 0x150B 成功后立刻**收到 6 字节 `53 00 <MAC 后 4 字节>`。
//: 轮询 CCCD（BLE2902）做边沿检测 —— Bluedroid 会在 central 写 CCCD 时更新它的本地值。
static void pollSubscribeEdge() {
  bool sub = (g_cccd_notify != nullptr) && g_cccd_notify->getNotifications();
  if (sub && !g_prev_subscribed && g_chr_notify != nullptr) {
    uint8_t hello[6] = {0x53, 0x00, g_mac[2], g_mac[3], g_mac[4], g_mac[5]};
    g_chr_notify->setValue(hello, sizeof(hello));
    g_chr_notify->notify();
    logLine("0x150B subscribed -> device handshake 53 00 %02X %02X %02X %02X", g_mac[2], g_mac[3],
            g_mac[4], g_mac[5]);
  }
  g_prev_subscribed = sub;
}

static void pollBattery() {
  unsigned long now = millis();
  if (now - g_last_battery_ms < 30000UL) {
    return;
  }
  g_last_battery_ms = now;
  if (g_chr_battery == nullptr || g_cccd_battery == nullptr) {
    return;
  }
  if (!g_app_connected || !g_cccd_battery->getNotifications()) {
    return;
  }
  uint8_t battery = 100;
  g_chr_battery->setValue(&battery, 1);
  g_chr_battery->notify();
}

static void pollAdvertisingRestart() {
  if (!g_restart_adv || millis() < g_restart_adv_at) {
    return;
  }
  g_restart_adv = false;
  BLEDevice::startAdvertising();
  logLine("advertising restarted");
}

//: 状态行：强度 / 软上限 / 波形快照 + 计数（连上时 1 秒一条，空闲时 5 秒一条）
//: 逐帧的波形看 `#B0`（每 100ms 一条）；这条是"一眼看总体"的心跳
static void pollStatusLine() {
  unsigned long now = millis();
  unsigned long period = g_app_connected ? 1000UL : 5000UL;
  if (now - g_last_stat_ms < period) {
    return;
  }
  unsigned long elapsed = now - g_last_stat_ms;
  uint32_t b0_prev = g_b0_at_last_stat;
  g_b0_at_last_stat = g_stat_b0;
  g_last_stat_ms = now;
  const float b0_rate = (elapsed > 0) ? (float)(g_stat_b0 - b0_prev) * 1000.0f / (float)elapsed : 0.0f;
  char wave_a[64];
  char wave_b[64];
  formatWave(wave_a, sizeof(wave_a), g_host.wave(0));
  formatWave(wave_b, sizeof(wave_b), g_host.wave(1));
  logLine("STAT t=%lu A=%u/%u B=%u/%u link=A%d/B%d wA=%s wB=%s | b0=%u (%.1f/s) bf=%u b1=%u drop=%u "
          "down=%u logdrop=%u conn=%d sub=%d raw=%d verbose=%d",
          now, (unsigned)g_host.strength(0), (unsigned)g_host.softCap(0),
          (unsigned)g_host.strength(1), (unsigned)g_host.softCap(1), g_host.loopEnabled(0) ? 1 : 0,
          g_host.loopEnabled(1) ? 1 : 0, wave_a, wave_b, (unsigned)g_stat_b0, b0_rate,
          (unsigned)g_stat_bf, (unsigned)g_stat_b1, (unsigned)g_stat_drop,
          (unsigned)g_stat_down_bytes, (unsigned)g_stat_log_drop, g_app_connected ? 1 : 0,
          (g_cccd_notify != nullptr && g_cccd_notify->getNotifications()) ? 1 : 0,
          g_raw_uplink ? 1 : 0, g_verbose_b0 ? 1 : 0);
}

// ══════════════════════════════════════════════════════════════
void setup() {
  // 逐帧日志（每秒 10 行左右）+ '#' 行，TX 缓冲给大一点，别让记录卡住 BLE 回调
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(100);

  esp_read_mac(g_mac, ESP_MAC_BT);

  // 主机状态初始化（强度从 0 起，软上限读 NVS —— 真机的软上限断电保存）
  g_host.reset();
  loadPersistedSettings();

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(23);  // 真机实测 MTU = 23；20 字节的 B0 正好放得下
  BLEDevice::setCustomGapHandler(gapEventHandler);  // 额外订阅 GAP 事件（打印/请求连接参数）

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());
  buildGattTable();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  // 真机空闲广播：只有名字，**不带服务 UUID、不带厂商数据**
  adv->setScanResponse(false);  // false → 名字进主广播包（不靠扫描响应）
  BLEDevice::startAdvertising();

  logLine("dglab-esp32 boot: name=%s bt_mac=%02X:%02X:%02X:%02X:%02X:%02X", DEVICE_NAME, g_mac[0],
          g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5]);
  logLine("GATT: stack built-in 0x1800(GAP)/0x1801(GATT) + ours 0x180A/0x180C/0x2003/0x2004");
  logLine("caps: host_logic=1 (firmware owns strength+B1) soft_cap=%u/%u raw_uplink=%d verbose_b0=%d",
          (unsigned)g_host.softCap(0), (unsigned)g_host.softCap(1), g_raw_uplink ? 1 : 0,
          g_verbose_b0 ? 1 : 0);
  logLine("serial: up B0/BF/CC/CD | down CE(heartbeat) C0/C1(raw) C2/C3(verbose) | 115200 8N1");
  logLine("serial cmds: D0 ch value = set strength | D1 ch delta = step | D2 ch on = loop A/B");
  logLine("note: raw binary frames stay OFF until a PC heartbeat (0xCE) arrives; send C0/C1 to force");
}

void loop() {
  pollDownlink();
  flushB1Queue();
  notifyStatusIfDirty();
  pollSubscribeEdge();
  pollBattery();
  pollAdvertisingRestart();
  pollStatusLine();
  delay(1);  // 下行字节最多多等 1ms —— 对"回显延迟"是白捡的
}

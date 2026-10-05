/*
 * dglab-esp32 —— 在 ESP32 上复刻 DG-LAB 郊狼主机（Coyote）3.0 的 BLE 外设，
 * 并把 App 写进来的原始指令经串口原样转交给 PC 侧上位机。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 zsyfpv
 *
 * 本文件是 dglab-esp32 的一部分，以 GNU GPL v3.0（或更新版本）发布，详见随附的 LICENSE。
 *
 * 职责边界（只做"忠实还原一台主机"）：
 *   ✅ 复刻真机 GATT 全表 + 广播名 47L121000 + App 订阅 0x150B 后的固定握手
 *   ✅ 收 App 的 B0/BF → **原样**经串口上行（不解析、不重组）
 *   ✅ 收主机的 B1 → notify 到 0x150B
 *   ❌ 不解析协议语义、不做叠加/惩罚/网络 —— 那是 PC 侧上位机的事
 *
 * 权威依据：对郊狼主机 3.0 真机的 GATT 实测（实测结果优先于官方文档）
 * 接口契约：本文件下方的「串口帧协议」常量 + README.md
 *
 * 已实测的关键点（照做，别照官方文档改）：
 *   1. 0x150A 是 WriteWithoutResponse（不是普通 WRITE）；
 *   2. 0x180A/0x1501 必须返回 "0c 03"，App 靠它判定协议版本 V3；
 *   3. App 初始化流程：read(180A/1500) → read(180A/1501) → notify(180A/1500)
 *      → notify(180C/150B) → check(180C/150A) → notify(180A/2A59)。
 */

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <esp_system.h>

// ══════════════════════════════════════════════════════════════
// 串口帧协议（与 PC 侧上位机一致；契约见 README.md §4）
//   上行（ESP32 → 主机）  0xB0+19B(B0) | 0xBF+6B(BF) | 0xCC(已连) | 0xCD(已断)
//   下行（主机 → ESP32）  0xB1+3B(B1)  | 0xCE(心跳)
//   靠 HEAD 字节定长分帧，没有长度前缀、没有校验
// ══════════════════════════════════════════════════════════════
static const uint8_t UP_B0 = 0xB0;
static const uint8_t UP_BF = 0xBF;
static const uint8_t UP_CONNECTED = 0xCC;
static const uint8_t UP_DISCONNECTED = 0xCD;
static const uint8_t DOWN_B1 = 0xB1;
static const uint8_t DOWN_HEARTBEAT = 0xCE;

static const size_t LEN_B0 = 20;  // 0xB0 + 19B
static const size_t LEN_BF = 7;   // 0xBF + 6B
static const size_t LEN_B1 = 4;   // 0xB1 + 3B

// 真机广播名 / 设备名（0x1800 / 0x2A00）
static const char *DEVICE_NAME = "47L121000";
// 序列号前缀（真机 0x180A/0x2A25 读回 "CEB3%Q" + 本机 MAC）
static const char *SERIAL_PREFIX = "CEB3%Q";

// ══════════════════════════════════════════════════════════════
// 受控侧状态
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

static volatile bool g_app_connected = false;
static bool g_prev_subscribed = false;
static unsigned long g_last_battery_ms = 0;
static bool g_restart_adv = false;
static unsigned long g_restart_adv_at = 0;
//: 是否已经为当前这次连接请求过快速连接参数（避免反复请求）
static bool g_conn_param_requested = false;

//: 诊断统计（排障用：一眼看出上行/下行到底走了多少字节、有没有解析成 B1）
static uint32_t g_up_frames = 0;      // 上行转发出去的帧数（B0/BF）
static uint32_t g_write_drops = 0;    // 长度/帧头不符、被丢弃的写入
static uint32_t g_down_bytes = 0;     // 从主机收到的下行字节数
static uint32_t g_b1_frames = 0;      // 下行里成功解析出的 B1 帧数
static uint32_t g_down_unknown = 0;   // 下行里等待帧头时被丢弃的字节数
static unsigned long g_last_stat_ms = 0;

// ══════════════════════════════════════════════════════════════
// 日志：**统一 '#' 前缀 + 换行**（PC 侧按行整行丢弃，不会污染二进制帧流）
// ══════════════════════════════════════════════════════════════
static void logLine(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print('#');
  Serial.print(buf);
  Serial.print('\n');
}

static void sendUplink(const uint8_t *data, size_t len) {
  Serial.write(data, len);
}

//: 把一串字节打成 hex 附在日志里（最多 24 字节）—— 排障时"App 到底写了什么"最有用
static void logHexBytes(const char *tag, const uint8_t *p, size_t len) {
  char buf[3 * 24 + 1];
  size_t n = (len < 24) ? len : 24;
  size_t off = 0;
  for (size_t i = 0; i < n && off + 4 < sizeof(buf); ++i) {
    off += (size_t)snprintf(buf + off, sizeof(buf) - off, "%02X ", p[i]);
  }
  if (off > 0) {
    buf[off - 1] = '\0';  // 去掉末尾空格
  } else {
    buf[0] = '\0';
  }
  logLine("%s len=%u %s%s", tag, (unsigned)len, buf, (len > n) ? " …" : "");
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
  logLine("连接参数：interval=%u (%.1f ms) latency=%u timeout=%u status=%d",
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
  logLine("请求快速连接参数（15~30ms / 无延迟）：rc=%d", (int)rc);
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
    logLine("App 已连接（conn_id=%d）", server->getConnId());
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    g_app_connected = false;
    g_prev_subscribed = false;
    uint8_t evt = UP_DISCONNECTED;
    sendUplink(&evt, 1);
    logLine("App 已断开 → 重新广播");
    // 断开后必须自己重启广播，否则手机再也搜不到（验收 A9）
    g_restart_adv = true;
    g_restart_adv_at = millis() + 300;
  }
};

//: 0x180C/0x150A（App 的所有指令）与未知服务的写入：只做**原样透传**与记录
class CoyoteWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    const uint8_t *p = chr->getData();
    size_t len = chr->getLength();
    if (p == nullptr || len == 0) {
      return;
    }
    if (len == LEN_B0 && p[0] == UP_B0) {
      updateStatusFromB0(p, len);
      sendUplink(p, len);
      ++g_up_frames;
      return;
    }
    if (len == LEN_BF && p[0] == UP_BF) {
      sendUplink(p, len);
      ++g_up_frames;
      return;
    }
    // 不按契约长发的写入不进链路（免得污染 PC 侧的定长分帧），但要**记下来**：
    // App 的其它写入 / 分包写入全靠这条日志才看得见
    ++g_write_drops;
    logHexBytes("丢弃异常写入", p, len);
  }

 private:
  //: 0x2A59 近似复刻：字节 2/3 跟着 B0 里 A/B 通道的首个波形强度走。
  //: 真机的精确换算规则仍未解出（见 README.md「已知问题」第 1 条）。
  static void updateStatusFromB0(const uint8_t *p, size_t len) {
    if (len < LEN_B0) {
      return;
    }
    uint8_t amp_a = p[8];   // A 通道波形强度第 1 组
    uint8_t amp_b = p[16];  // B 通道波形强度第 1 组
    g_status[2] = (amp_a > 100) ? 0 : amp_a;  // >100 → 该通道整组作废
    g_status[3] = (amp_b > 100) ? 0 : amp_b;
    if (g_chr_status != nullptr) {
      g_chr_status->setValue(g_status, sizeof(g_status));
      if (g_cccd_status != nullptr && g_cccd_status->getNotifications()) {
        g_chr_status->notify();
      }
    }
  }
};

class UnknownWriteLogCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    const uint8_t *p = chr->getData();
    size_t len = chr->getLength();
    if (len == 0) {
      return;
    }
    logLine("未知特征 0x%s 写入 %u 字节 head=0x%02X", chr->getUUID().toString().c_str(),
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
// 下行（主机 → ESP32）：0xB1 → notify 0x150B；0xCE → 回一条 '#' 日志（在线探测应答）
// ══════════════════════════════════════════════════════════════
static uint8_t g_down_buf[LEN_B1];
static size_t g_down_got = 0;
static size_t g_down_need = 0;
//: 半帧看门狗：收到帧头后迟迟凑不齐 → 丢弃重同步（别让丢一个字节把下行永久卡死）
static unsigned long g_down_last_ms = 0;
static const unsigned long DOWN_FRAME_TIMEOUT_MS = 200;

static void handleDownlinkFrame() {
  if (g_down_buf[0] != DOWN_B1) {
    logLine("下行帧头异常 0x%02X，丢弃", (unsigned)g_down_buf[0]);
    return;
  }
  ++g_b1_frames;
  logLine("B1 ← 主机：%02X %02X %02X %02X", (unsigned)g_down_buf[0], (unsigned)g_down_buf[1],
          (unsigned)g_down_buf[2], (unsigned)g_down_buf[3]);
  if (g_chr_notify == nullptr) {
    return;
  }
  g_chr_notify->setValue(g_down_buf, LEN_B1);
  if (g_cccd_notify != nullptr && g_cccd_notify->getNotifications()) {
    g_chr_notify->notify();
  } else {
    logLine("B1 无订阅者（CCCD 未置位），丢弃");
  }
}

static void pollDownlink() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    ++g_down_bytes;
    g_down_last_ms = millis();
    if (g_down_need == 0) {
      if (b == DOWN_B1) {
        // ★ 帧头也算这一帧的一部分（早先这里少算了 1 字节 → B1 永远凑不满、被吞掉）
        g_down_buf[0] = b;
        g_down_got = 1;
        g_down_need = LEN_B1;
      } else if (b == DOWN_HEARTBEAT) {
        // ★ 心跳：回一条 '#' 日志行。PC 侧用它做"线另一端是不是假主机固件"的在线探测，
        //   正因为 '#' 行不在帧空间里，不占用/不污染协议（PC 侧按行丢弃）。
        logLine("alive（心跳 0xCE）");
      } else if (++g_down_unknown <= 8) {
        // 只报前几次（开机时 ROM 日志的残渣会有一串，别刷屏）
        logLine("下行未知字节 0x%02X（等帧头，丢弃）", (unsigned)b);
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
    logLine("下行半帧超时（%u/%u 字节）→ 丢弃重同步", (unsigned)g_down_got, (unsigned)g_down_need);
    g_down_need = 0;
    g_down_got = 0;
  }
}

//: 周期性自报统计（排障时一眼看出：上行有没有转发、下行有没有字节、B1 有没有解析出来）
static void pollStats() {
  unsigned long now = millis();
  if (now - g_last_stat_ms < 8000UL) {
    return;
  }
  g_last_stat_ms = now;
  logLine("stats up=%u drops=%u down_bytes=%u b1=%u unknown=%u conn=%d sub=%d",
          (unsigned)g_up_frames, (unsigned)g_write_drops, (unsigned)g_down_bytes,
          (unsigned)g_b1_frames, (unsigned)g_down_unknown, g_app_connected ? 1 : 0,
          (g_cccd_notify != nullptr && g_cccd_notify->getNotifications()) ? 1 : 0);
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
    logLine("0x150B 已被订阅 → 发设备握手 53 00 %02X %02X %02X %02X", g_mac[2], g_mac[3],
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
  logLine("已重新开始广播");
}

// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);

  esp_read_mac(g_mac, ESP_MAC_BT);

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

  logLine("假郊狼主机启动：name=%s bt_mac=%02X:%02X:%02X:%02X:%02X:%02X", DEVICE_NAME, g_mac[0],
          g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5]);
  logLine("GATT：栈内建 0x1800(GAP)/0x1801(GATT) + 自建 0x180A/0x180C/0x2003/0x2004");
  logLine("链路就绪：上行 B0/BF/CC/CD，下行 B1/CE，115200 8N1");
}

void loop() {
  pollDownlink();
  pollSubscribeEdge();
  pollBattery();
  pollAdvertisingRestart();
  pollStats();
  delay(1);  // 下行字节最多多等 1ms（原 2ms）—— 对"回显延迟"是白捡的
}

// 纯逻辑单测：src/host_state.h（用 g++ 直接跑，不需要板子、不需要 PlatformIO）
//
// 跑法（在 firmware/ 目录下）：
//   g++ -std=c++11 -Wall -Wextra -I src -o test_host_state tools/test_host_state.cpp && ./test_host_state
#include "host_state.h"

#include <cstdio>
#include <cstring>
#include <string>

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (cond) {                                                              \
      ++g_pass;                                                              \
    } else {                                                                 \
      ++g_fail;                                                              \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
    }                                                                        \
  } while (0)

#define CHECK_EQ(a, b)                                                       \
  do {                                                                       \
    long long _x = (long long)(a), _y = (long long)(b);                      \
    if (_x == _y) {                                                          \
      ++g_pass;                                                              \
    } else {                                                                 \
      ++g_fail;                                                              \
      std::printf("FAIL %s:%d  %s=%lld != %s=%lld\n", __FILE__, __LINE__, #a, \
                  _x, #b, _y);                                               \
    }                                                                        \
  } while (0)

// 造一条 B0：seq | modeA | modeB | setA | setB | 波形
static void makeB0(uint8_t *out, uint8_t seq, uint8_t mode_a, uint8_t mode_b, uint8_t set_a,
                   uint8_t set_b, const uint8_t freqA[4], const uint8_t ampA[4],
                   const uint8_t freqB[4], const uint8_t ampB[4]) {
  std::memset(out, 0, 20);
  out[0] = dglab::HEAD_B0;
  out[1] = (uint8_t)(((seq & 0x0F) << 4) | ((mode_a & 3) << 2) | (mode_b & 3));
  out[2] = set_a;
  out[3] = set_b;
  std::memcpy(out + 4, freqA, 4);
  std::memcpy(out + 8, ampA, 4);
  std::memcpy(out + 12, freqB, 4);
  std::memcpy(out + 16, ampB, 4);
}

int main() {
  using namespace dglab;

  const uint8_t f[4] = {10, 10, 20, 30};
  const uint8_t a[4] = {0, 5, 10, 50};

  // ── 1. 文档 No.2：当前 A=10，B0 设置 A+5（mode ADD，seq=0）────────────────
  {
    Host h;
    uint8_t fr[20];
    makeB0(fr, 0, MODE_SET, MODE_NONE, 10, 0, f, a, f, a);
    B0Result r = h.applyB0(fr, 20);
    CHECK(r.ok);
    CHECK_EQ(h.strength(0), 10);
    CHECK(!r.wants_reply());  // seq=0 → 不回 B1

    makeB0(fr, 0, MODE_ADD, MODE_NONE, 5, 0, f, a, f, a);
    r = h.applyB0(fr, 20);
    CHECK(r.changed_a);
    CHECK_EQ(h.strength(0), 15);
    CHECK(!r.wants_reply());
  }

  // ── 2. seq≠0 必回 B1（哪怕强度没变）──────────────────────────────────────
  {
    Host h;
    uint8_t fr[20];
    makeB0(fr, 3, MODE_SET, MODE_NONE, 10, 0, f, a, f, a);
    B0Result r = h.applyB0(fr, 20);
    CHECK(r.wants_reply());
    CHECK_EQ(r.seq, 3);
    uint8_t b1[4];
    h.buildB1(r.seq, b1);
    CHECK_EQ(b1[0], 0xB1);
    CHECK_EQ(b1[1], 3);
    CHECK_EQ(b1[2], 10);
    CHECK_EQ(b1[3], 0);

    // 原样再来一条（强度没变，但 seq≠0）→ 仍然要回
    makeB0(fr, 4, MODE_NONE, MODE_NONE, 0, 0, f, a, f, a);
    r = h.applyB0(fr, 20);
    CHECK(!r.changed_a && !r.changed_b);
    CHECK(r.wants_reply());
  }

  // ── 3. 越界设定值按 0 处理（不是夹到 200！）──────────────────────────────
  {
    Host h;
    uint8_t fr[20];
    makeB0(fr, 1, MODE_SET, MODE_SET, 20, 30, f, a, f, a);
    h.applyB0(fr, 20);
    CHECK_EQ(h.strength(0), 20);
    CHECK_EQ(h.strength(1), 30);
    // A 通道 SET 201 → 按 0；B 通道 ADD 201 → 按 +0（不变）
    makeB0(fr, 2, MODE_SET, MODE_ADD, 201, 201, f, a, f, a);
    h.applyB0(fr, 20);
    CHECK_EQ(h.strength(0), 0);
    CHECK_EQ(h.strength(1), 30);
  }

  // ── 4. SUB 不会穿到负数 ──────────────────────────────────────────────────
  {
    Host h;
    uint8_t fr[20];
    makeB0(fr, 1, MODE_SET, MODE_NONE, 5, 0, f, a, f, a);
    h.applyB0(fr, 20);
    makeB0(fr, 2, MODE_SUB, MODE_NONE, 200, 0, f, a, f, a);
    h.applyB0(fr, 20);
    CHECK_EQ(h.strength(0), 0);
  }

  // ── 5. BF 软上限：夹住 B0，>200 不修改，收紧后立刻压回 ────────────────────
  {
    Host h;
    uint8_t bf[7] = {HEAD_BF, 150, 30, 0, 0, 0, 0};
    BfResult br = h.applyBf(bf, 7);
    CHECK(br.ok);
    CHECK(br.cap_changed);
    CHECK_EQ(h.softCap(0), 150);
    CHECK_EQ(h.softCap(1), 30);

    uint8_t fr[20];
    makeB0(fr, 1, MODE_SET, MODE_SET, 200, 200, f, a, f, a);
    h.applyB0(fr, 20);
    CHECK_EQ(h.strength(0), 150);  // 软上限夹住
    CHECK_EQ(h.strength(1), 30);

    // 收紧 A 到 50 → 当前 150 立刻压回 50
    uint8_t bf2[7] = {HEAD_BF, 50, 30, 7, 8, 9, 10};
    br = h.applyBf(bf2, 7);
    CHECK(br.cap_changed && br.clamped);
    CHECK_EQ(h.strength(0), 50);
    CHECK_EQ(h.freqBalance(0), 7);
    CHECK_EQ(h.freqBalance(1), 8);
    CHECK_EQ(h.ampBalance(0), 9);
    CHECK_EQ(h.ampBalance(1), 10);

    // 软上限 > 200 → 文档：不修改
    uint8_t bf3[7] = {HEAD_BF, 201, 201, 0, 0, 0, 0};
    br = h.applyBf(bf3, 7);
    CHECK(!br.cap_changed);
    CHECK_EQ(h.softCap(0), 50);
    CHECK_EQ(h.softCap(1), 30);
  }

  // ── 6. 波形合法性：任一越界 → 该通道整组作废 ──────────────────────────────
  {
    Host h;
    uint8_t fr[20];
    makeB0(fr, 0, MODE_NONE, MODE_NONE, 0, 0, f, a, f, a);
    h.applyB0(fr, 20);
    CHECK(h.wave(0).valid);
    CHECK(h.wave(1).valid);

    // A 通道第 4 组强度 101 → A 整组作废，B 不受影响
    const uint8_t bad_amp[4] = {0, 5, 10, 101};
    makeB0(fr, 0, MODE_NONE, MODE_NONE, 0, 0, f, bad_amp, f, a);
    h.applyB0(fr, 20);
    CHECK(!h.wave(0).valid);
    CHECK(h.wave(1).valid);

    // 频率 9 越界（<10）→ 整组作废
    const uint8_t bad_freq[4] = {9, 10, 20, 30};
    makeB0(fr, 0, MODE_NONE, MODE_NONE, 0, 0, bad_freq, a, f, a);
    h.applyB0(fr, 20);
    CHECK(!h.wave(0).valid);

    // 频率 241 越界（>240）
    const uint8_t bad_freq2[4] = {241, 10, 20, 30};
    makeB0(fr, 0, MODE_NONE, MODE_NONE, 0, 0, bad_freq2, a, f, a);
    h.applyB0(fr, 20);
    CHECK(!h.wave(0).valid);
  }

  // ── 7. 长度 / HEAD 不符 → 什么都不改 ─────────────────────────────────────
  {
    Host h;
    uint8_t fr[20];
    makeB0(fr, 1, MODE_SET, MODE_SET, 40, 40, f, a, f, a);
    B0Result r = h.applyB0(fr, 19);  // 短了
    CHECK(!r.ok);
    CHECK_EQ(h.strength(0), 0);
    fr[0] = 0x50;  // 不是 B0
    r = h.applyB0(fr, 20);
    CHECK(!r.ok);
    r = h.applyB0(nullptr, 20);
    CHECK(!r.ok);
  }

  // ── 8. 格式化出来的日志字段是人能看懂的 ──────────────────────────────────
  {
    Host h;
    uint8_t fr[20];
    makeB0(fr, 0, MODE_NONE, MODE_NONE, 0, 0, f, a, f, a);
    h.applyB0(fr, 20);
    char buf[64];
    Host::formatWave(buf, sizeof(buf), h.wave(0));
    CHECK(std::string(buf) == "10:0,10:5,20:10,30:50");

    const uint8_t off_amp[4] = {0, 0, 0, 101};
    makeB0(fr, 0, MODE_NONE, MODE_NONE, 0, 0, f, off_amp, f, a);
    h.applyB0(fr, 20);
    Host::formatWave(buf, sizeof(buf), h.wave(0));
    // 越界也照样打数字，只是前面加 '!' —— "为什么没波形"要一眼看得出来
    CHECK(std::string(buf) == "!10:0,10:0,20:0,30:101");

    CHECK_EQ(Host::modeChar(MODE_NONE), '.');
    CHECK_EQ(Host::modeChar(MODE_ADD), '+');
    CHECK_EQ(Host::modeChar(MODE_SUB), '-');
    CHECK_EQ(Host::modeChar(MODE_SET), '=');
  }

  // ── 9. 真实 App 报文（官方 example.md 第 1 条）───────────────────────────
  {
    Host h;
    const uint8_t raw[20] = {0xB0, 0x00, 0x00, 0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0x00, 0x0A,
                             0x14, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65};
    B0Result r = h.applyB0(raw, 20);
    CHECK(r.ok);
    CHECK_EQ(r.seq, 0);
    CHECK_EQ(r.mode_a, MODE_NONE);
    CHECK_EQ(h.strength(0), 0);
    CHECK(h.wave(0).valid);   // A 通道 {10,10,10,10}/{0,10,20,30} → 有效
    CHECK(!h.wave(1).valid);  // B 通道 {0,0,0,0}/{0,0,0,101} → 作废（静音开关）
  }

  // ── 10. 网页/拨轮直接改强度（manualStrength）：语义与 B0 一致，受软上限夹值 ──
  {
    Host h;
    CHECK(h.manualStrength(0, MODE_SET, 30));
    CHECK_EQ(h.strength(0), 30);
    CHECK(!h.manualStrength(0, MODE_SET, 30));  // 值没变 → 返回 false（不回 B1）
    CHECK(h.manualStrength(0, MODE_ADD, 1));
    CHECK_EQ(h.strength(0), 31);
    CHECK(h.manualStrength(0, MODE_SUB, 1));
    CHECK_EQ(h.strength(0), 30);
    // 减过头不会变成负数
    CHECK(h.manualStrength(0, MODE_SUB, 200));
    CHECK_EQ(h.strength(0), 0);
    // 软上限照样夹住
    uint8_t bf[7] = {HEAD_BF, 40, 200, 0, 0, 0, 0};
    h.applyBf(bf, 7);
    h.manualStrength(1, MODE_SET, 200);
    CHECK_EQ(h.strength(1), 200);  // B 通道上限 200
    h.manualStrength(0, MODE_ADD, 200);
    CHECK_EQ(h.strength(0), 40);   // A 通道上限 40
  }

  // ── 11. 回路开关（A/B 各自独立，默认闭合）────────────────────────────────
  {
    Host h;
    CHECK(h.loopEnabled(0));
    CHECK(h.loopEnabled(1));
    CHECK(h.setLoop(0, false));
    CHECK(!h.loopEnabled(0));
    CHECK(h.loopEnabled(1));       // B 不受影响
    CHECK(!h.setLoop(0, false));   // 已经是断开 → 返回 false
    CHECK(h.setLoop(0, true));
    CHECK(h.loopEnabled(0));
    h.reset();                     // 断电重启后回到默认闭合
    CHECK(h.loopEnabled(0) && h.loopEnabled(1));
    CHECK_EQ(h.strength(0), 0);
    CHECK_EQ(h.softCap(0), STRENGTH_MAX);
  }

  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}

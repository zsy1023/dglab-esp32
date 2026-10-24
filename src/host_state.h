/*
 * host_state.h —— 郊狼主机 3.0 的**纯逻辑**：B0 / BF → 通道强度、软上限、波形。
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 zsyfpv
 *
 * 为什么单独一个头文件：这里**一行 Arduino / BLE 代码都没有**，所以可以拿普通 g++
 * 直接跑单测（任务书 §8 的硬性要求：协议编解码 / 状态推进必须能脱离硬件测试）。
 * 固件里 main.cpp 只负责「收字节 → 调这里 → 打日志 / notify」。
 *
 * 权威依据（冲突时以真机为准）：
 *   * 官方逐字节规格 reference/dglab-bluetooth-protocol/coyote/v3/README.md
 *   * 真机实测 docs/research/郊狼真机GATT实测.md
 *
 * 关键语义（都踩过，别"顺手优化"）：
 *   1. B0 的 ADD / SUB 是**相对**量 —— 不知道当前强度就算不出新强度；
 *   2. 通道强度 0~200，越界的设定值**按 0 处理**（不是夹到 200！）；
 *   3. 软上限（BF 设定、断电保存）会同时限制 B0 与任何拨轮操作；
 *   4. 波形频率 10~240、波形强度 0~100，**任一取值越界 → 该通道 4 组整组作废**；
 *   5. 序列号 ≠ 0 → 必须回一条同序列号的 B1（哪怕强度一个字节都没变）。
 */

#ifndef DGLAB_HOST_STATE_H
#define DGLAB_HOST_STATE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace dglab {

// ── 指令头 / 长度 ──────────────────────────────────────────────
constexpr uint8_t HEAD_B0 = 0xB0;
constexpr uint8_t HEAD_BF = 0xBF;
constexpr uint8_t HEAD_B1 = 0xB1;
constexpr size_t B0_LEN = 20;
constexpr size_t BF_LEN = 7;
constexpr size_t B1_LEN = 4;

// ── 取值范围 ──────────────────────────────────────────────────
constexpr uint8_t STRENGTH_MAX = 200;  // 通道强度 0~200
constexpr uint8_t WAVE_AMP_MAX = 100;  // 波形强度 0~100
constexpr uint8_t FREQ_MIN = 10;       // 波形频率 10~240
constexpr uint8_t FREQ_MAX = 240;
constexpr int WAVE_GROUP = 4;          // 每通道 4 组「频率:强度」，每组 25ms，共 100ms

// ── 强度值解读方式（B0 byte1 低 4bit：高 2bit = A 通道，低 2bit = B 通道）──
constexpr uint8_t MODE_NONE = 0b00;  // 该通道强度不变（设定值无效）
constexpr uint8_t MODE_ADD = 0b01;   // 相对增加
constexpr uint8_t MODE_SUB = 0b10;   // 相对减少
constexpr uint8_t MODE_SET = 0b11;   // 绝对设置

//: 一个通道的 4 组波形（App 每 100ms 换一批）
struct ChannelWave {
  uint8_t freq[WAVE_GROUP];
  uint8_t amp[WAVE_GROUP];
  bool valid;  // false = 整组作废：真机不输出该通道波形
};

//: 一条 B0 应用后的结果（给固件打日志 / 决定要不要回 B1 用）
struct B0Result {
  bool ok;            // 长度 / HEAD 不符 → false（什么都没改）
  uint8_t seq;        // 序列号 0~15
  uint8_t mode_a;     // A 通道解读方式
  uint8_t mode_b;     // B 通道解读方式
  uint8_t setpoint_a; // A 通道强度设定值（原样，未经处理）
  uint8_t setpoint_b; // B 通道强度设定值
  bool changed_a;     // A 通道强度真的变了
  bool changed_b;     // B 通道强度真的变了

  //: 真机实测：序列号 ≠ 0 就必回一条 B1（不看强度是否真的变了）
  bool wants_reply() const { return ok && seq != 0; }
};

//: 一条 BF 应用后的结果
struct BfResult {
  bool ok;           // 长度 / HEAD 不符 → false
  bool cap_changed;  // 软上限变了（需要写 NVS 持久化）
  bool clamped;      // 软上限收紧后，当前强度被压回上限
};

//: 一台「郊狼主机」的受控侧状态
class Host {
 public:
  Host() { reset(); }

  //: 断电重启：强度回 0，软上限回 200（随后用 setSoftCaps 从 NVS 恢复）
  void reset() {
    for (int ch = 0; ch < 2; ++ch) {
      strength_[ch] = 0;
      soft_cap_[ch] = STRENGTH_MAX;
      freq_bal_[ch] = 0;
      amp_bal_[ch] = 0;
      loop_[ch] = true;  // 回路默认闭合（模拟器没有输出级，这只是状态位）
      for (int i = 0; i < WAVE_GROUP; ++i) {
        wave_[ch].freq[i] = 0;
        wave_[ch].amp[i] = 0;
      }
      wave_[ch].valid = false;
    }
  }

  // ── 读状态 ────────────────────────────────────────────────
  uint8_t strength(int ch) const { return strength_[ch & 1]; }
  uint8_t softCap(int ch) const { return soft_cap_[ch & 1]; }
  uint8_t freqBalance(int ch) const { return freq_bal_[ch & 1]; }
  uint8_t ampBalance(int ch) const { return amp_bal_[ch & 1]; }
  const ChannelWave &wave(int ch) const { return wave_[ch & 1]; }
  //: 回路（输出）是否闭合。断开时真机上不会有电流，波形等于不输出。
  bool loopEnabled(int ch) const { return loop_[ch & 1]; }

  //: 从 NVS 恢复软上限（真机断电保存；越界值不采信）
  void setSoftCaps(uint8_t cap_a, uint8_t cap_b) {
    soft_cap_[0] = (cap_a <= STRENGTH_MAX) ? cap_a : STRENGTH_MAX;
    soft_cap_[1] = (cap_b <= STRENGTH_MAX) ? cap_b : STRENGTH_MAX;
    clampToCaps();
  }

  //: 外部（网页按钮 / 未来的物理拨轮）直接改强度，返回强度是否真的变了。
  //: ``mode`` 用 MODE_ADD / MODE_SUB / MODE_SET —— 与 B0 的语义、软上限夹值完全一致。
  bool manualStrength(int ch, uint8_t mode, uint8_t value) { return step(ch & 1, mode, value); }

  //: 回路开关（按通道独立），返回状态是否真的变了
  bool setLoop(int ch, bool on) {
    const int c = ch & 1;
    const bool changed = (loop_[c] != on);
    loop_[c] = on;
    return changed;
  }

  // ── 应用指令 ──────────────────────────────────────────────
  //: B0：强度（绝对 / 相对，受软上限夹值）+ 两通道波形
  B0Result applyB0(const uint8_t *frame, size_t len) {
    B0Result r = {};
    if (frame == nullptr || len < B0_LEN || frame[0] != HEAD_B0) {
      return r;
    }
    r.ok = true;
    r.seq = (uint8_t)((frame[1] >> 4) & 0x0F);
    r.mode_a = (uint8_t)((frame[1] >> 2) & 0x03);
    r.mode_b = (uint8_t)(frame[1] & 0x03);
    r.setpoint_a = frame[2];
    r.setpoint_b = frame[3];
    r.changed_a = step(0, r.mode_a, r.setpoint_a);
    r.changed_b = step(1, r.mode_b, r.setpoint_b);
    parseWave(&wave_[0], frame + 4, frame + 8);
    parseWave(&wave_[1], frame + 12, frame + 16);
    return r;
  }

  //: BF：软上限（>200 不修改）+ 频率/强度平衡参数；收紧软上限会立刻把强度压回去
  BfResult applyBf(const uint8_t *frame, size_t len) {
    BfResult r = {};
    if (frame == nullptr || len < BF_LEN || frame[0] != HEAD_BF) {
      return r;
    }
    r.ok = true;
    if (frame[1] <= STRENGTH_MAX && soft_cap_[0] != frame[1]) {
      soft_cap_[0] = frame[1];
      r.cap_changed = true;
    }
    if (frame[2] <= STRENGTH_MAX && soft_cap_[1] != frame[2]) {
      soft_cap_[1] = frame[2];
      r.cap_changed = true;
    }
    freq_bal_[0] = frame[3];
    freq_bal_[1] = frame[4];
    amp_bal_[0] = frame[5];
    amp_bal_[1] = frame[6];
    r.clamped = clampToCaps();
    return r;
  }

  //: B1 回包：`B1 | 序列号 | A 实际强度 | B 实际强度`（4 字节）
  size_t buildB1(uint8_t seq, uint8_t out[B1_LEN]) const {
    out[0] = HEAD_B1;
    out[1] = seq;
    out[2] = strength_[0];
    out[3] = strength_[1];
    return B1_LEN;
  }

  // ── 格式化（日志用；纯 ASCII）──────────────────────────────
  //: 解读方式 → 一个字符：'.' 不变 / '+' 增加 / '-' 减少 / '=' 绝对设置
  static char modeChar(uint8_t mode) {
    switch (mode) {
      case MODE_ADD:
        return '+';
      case MODE_SUB:
        return '-';
      case MODE_SET:
        return '=';
      default:
        return '.';
    }
  }

  //: 4 组波形压成 "freq:amp,freq:amp,..."（逗号分隔 = 一个 token，网页/上位机好解析）；
  //: **整组作废时前面加 `!`**（真机规则：任一 freq 不在 10~240 或任一 amp >100 → 该通道不输出）。
  //: 越界了也照样把数字打出来 —— "为什么没有波形"必须一眼看得出来，而不是只剩一个 off。
  static void formatWave(char *out, size_t cap, const ChannelWave &w) {
    if (out == nullptr || cap == 0) {
      return;
    }
    size_t off = 0;
    if (!w.valid) {
      int n = snprintf(out, cap, "!");
      if (n <= 0) {
        return;
      }
      off = (size_t)n;
    }
    for (int i = 0; i < WAVE_GROUP && off + 1 < cap; ++i) {
      int m = snprintf(out + off, cap - off, "%s%u:%u", (i == 0) ? "" : ",", (unsigned)w.freq[i],
                       (unsigned)w.amp[i]);
      if (m <= 0) {
        break;
      }
      off += (size_t)m;
    }
  }

 private:
  //: 推进一个通道的强度，返回是否真的变了
  bool step(int ch, uint8_t mode, uint8_t setpoint) {
    if (mode == MODE_NONE) {
      return false;  // 该通道强度不变，设定值无论是什么都无效
    }
    const uint8_t v = (setpoint > STRENGTH_MAX) ? 0 : setpoint;  // 越界设定值按 0 处理
    int next = (int)strength_[ch];
    if (mode == MODE_ADD) {
      next += (int)v;
    } else if (mode == MODE_SUB) {
      next -= (int)v;
    } else {  // MODE_SET
      next = (int)v;
    }
    if (next < 0) {
      next = 0;
    }
    if (next > (int)soft_cap_[ch]) {
      next = (int)soft_cap_[ch];  // 软上限：怎么调都不会超
    }
    const bool changed = ((uint8_t)next != strength_[ch]);
    strength_[ch] = (uint8_t)next;
    return changed;
  }

  //: 波形合法性：任一频率越界或任一强度 >100 → 整组作废（真机规则）
  static void parseWave(ChannelWave *w, const uint8_t *freq, const uint8_t *amp) {
    bool valid = true;
    for (int i = 0; i < WAVE_GROUP; ++i) {
      w->freq[i] = freq[i];
      w->amp[i] = amp[i];
      if (freq[i] < FREQ_MIN || freq[i] > FREQ_MAX || amp[i] > WAVE_AMP_MAX) {
        valid = false;
      }
    }
    w->valid = valid;
  }

  //: 软上限变小后把当前强度压回去；返回是否发生了夹值
  bool clampToCaps() {
    bool clamped = false;
    for (int ch = 0; ch < 2; ++ch) {
      if (strength_[ch] > soft_cap_[ch]) {
        strength_[ch] = soft_cap_[ch];
        clamped = true;
      }
    }
    return clamped;
  }

  uint8_t strength_[2];
  uint8_t soft_cap_[2];
  uint8_t freq_bal_[2];
  uint8_t amp_bal_[2];
  bool loop_[2];  // 回路（输出）是否闭合 —— 真机上断开就没有电流
  ChannelWave wave_[2];
};

}  // namespace dglab

#endif  // DGLAB_HOST_STATE_H

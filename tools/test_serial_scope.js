// serial_scope.html 的解析/状态机单测（不需要浏览器、不需要板子，只要 node）
//
// 跑法（在 firmware/ 目录下）：
//   node tools/test_serial_scope.js tools/serial_scope.html
//
// 原理：把页面里的 <script> 抠出来，用 vm 在一个打桩的 DOM 里跑（Proxy 伪装 document/window/navigator），
// 然后把固件真实的 '#' 日志行喂给 handleLine()，断言状态与波形历史。
const fs = require("fs");
const vm = require("vm");
const assert = require("assert");

const htmlPath = process.argv[2] || "tools/serial_scope.html";
const html = fs.readFileSync(htmlPath, "utf8");
const m = /<script>([\s\S]*)<\/script>/.exec(html);
if (!m) throw new Error("no <script>");
const code = m[1];

function fake(name) {
  const store = {
    dataset: { ch: name === "card1" ? "1" : "0" },
    style: {},
    classList: { add() {}, remove() {}, toggle() {} },
    children: [],
    childElementCount: 0,
    firstElementChild: null,
    textContent: "",
    innerHTML: "",
    checked: false,
    value: "1",
    clientWidth: 400,
    scrollTop: 0,
    scrollHeight: 0,
  };
  const target = {
    querySelector: () => fake("child"),
    querySelectorAll: () => [],
    addEventListener() {},
    appendChild(el) { store.children.push(el); },
    removeChild() {},
    getContext: () => ctx2d(),
    setAttribute() {},
    getAttribute: () => null,
  };
  return new Proxy(store, {
    get(t, k) {
      if (k in t) return t[k];
      if (k in target) return target[k];
      if (typeof k === "symbol") return undefined;
      return () => {};
    },
  });
}
function ctx2d() {
  return new Proxy({}, {
    get(t, k) {
      if (k === "createLinearGradient") return () => ({ addColorStop() {} });
      if (k === "measureText") return () => ({ width: 10 });
      if (k in t) return t[k];
      return () => {};
    },
    set(t, k, v) { t[k] = v; return true; },
  });
}

const cards = [fake("card0"), fake("card1")];
const documentStub = {
  querySelector: () => fake("el"),
  querySelectorAll: (sel) => (sel === ".card" ? cards : []),
  createElement: () => fake("new"),
  activeElement: null,
  addEventListener() {},
};
const ctx = {
  document: documentStub,
  navigator: { serial: { addEventListener() {} } },
  window: { addEventListener() {}, devicePixelRatio: 1 },
  TextDecoder,
  Uint8Array,
  Number,
  Math,
  console: { log() {}, warn() {}, error() {} },
  setTimeout,
  clearTimeout,
  setInterval,
  clearInterval,
};
ctx.globalThis = ctx;
const codeTest = code + "\n;globalThis.__t = { state, cards, handleLine, tokens, parseWaveCell };";
vm.createContext(ctx);
vm.runInContext(codeTest, ctx, { filename: "serial_scope-inline.js" });

const { handleLine, tokens, parseWaveCell, state, cards: scopeCards } = ctx.__t;
assert.strictEqual(typeof handleLine, "function", "handleLine should be global");
assert.strictEqual(scopeCards[0].el, cards[0], "script cards wrap the stubbed elements");

const eqArr = (a, b, msg) => assert.strictEqual(JSON.stringify(a), JSON.stringify(b), msg);
const feed = (line) => handleLine(line);

// 1. 逐帧 B0
feed("#B0 t=1520 seq=0 mode=A./B. set=0/0 A=5/150 B=0/30 link=A1/B0 wA=10:0,10:10,20:20,30:30 wB=off");
eqArr(state.strength, [5, 0], "strength parse");
eqArr(state.cap, [150, 30], "cap parse");
eqArr(state.loop, [true, false], "link parse A1/B0");
assert.strictEqual(scopeCards[0].hist.length, 4, "A channel should get 4 groups");
assert.strictEqual(scopeCards[1].hist.length, 4, "B channel should get 4 groups (off -> zeros)");
eqArr(scopeCards[0].hist, [0, 10, 20, 30], "A wave amps");
eqArr(scopeCards[1].hist, [0, 0, 0, 0], "B off -> zeros");
eqArr(scopeCards[0].histFreq, [10, 10, 20, 30], "A wave freqs");

// 2. 再喂一帧
feed("#B0 t=1620 seq=1 mode=A+/B. set=5/0 A=10/150 B=0/30 link=A0/B1 wA=10:50,10:60,20:70,30:80 wB=10:1,10:2,10:3,10:4");
assert.strictEqual(scopeCards[0].hist.length, 8, "history accumulates");
eqArr(state.loop, [false, true], "link flips");
eqArr(state.strength, [10, 0]);

// 3. STAT
feed("#STAT t=2620 A=10/150 B=0/30 link=A1/B1 wA=10:0,10:10,20:20,30:30 wB=off | b0=57 (10.0/s) bf=1 b1=3 drop=0 down=0 logdrop=0 conn=1 sub=1 raw=0 verbose=1");
assert.strictEqual(state.stat.conn, 1, "conn");
assert.strictEqual(state.stat.sub, 1, "sub");
assert.strictEqual(state.stat.rate, 10, "rate");
eqArr(state.loop, [true, true], "STAT link");

// 4. 强度回显
feed("STRENGTH A=11/150 B=0/30 (via serial cmd 0xD1)");
eqArr(state.strength, [11, 0], "serial cmd echo");
feed("#STRENGTH A=12/150 B=0/30 (seq=3)");
eqArr(state.strength, [12, 0], "app-driven echo");

// 5. 回路状态行
feed("#link: loop A=closed B=open | strength A=12/150 B=0/30");
eqArr(state.loop, [true, false], "link: loop line");

// 6. 杂项行
feed("#B1 -> App seq=0 A=12 B=0");
feed("#IGNORED (not a host B0/BF opcode) len=9 50 02 00 00 00 00 00 00 00");
feed("#dglab-esp32 boot: name=47L121000 bt_mac=EC:E3:34:A9:13:0E");
feed("");
feed("   ");
feed("garbage binary \u0000\u0001");
eqArr(state.strength, [12, 0], "garbage must not change state");

// 7. tokens / parseWaveCell
assert.strictEqual(parseWaveCell("off"), null);
assert.strictEqual(parseWaveCell(""), null);
assert.strictEqual(parseWaveCell(null), null);
const cellOk = parseWaveCell("10:0,10:5,20:10,30:50");
eqArr(cellOk.groups, [{ f: 10, a: 0 }, { f: 10, a: 5 }, { f: 20, a: 10 }, { f: 30, a: 50 }]);
assert.strictEqual(cellOk.valid, true, "no ! prefix -> valid");
const cellBad = parseWaveCell("!10:0,10:0,20:0,30:101");
eqArr(cellBad.groups, [{ f: 10, a: 0 }, { f: 10, a: 0 }, { f: 20, a: 0 }, { f: 30, a: 101 }]);
assert.strictEqual(cellBad.valid, false, "! prefix -> invalid (firmware discarded the group)");
assert.strictEqual(parseWaveCell("10:0,10:5"), null, "too few groups -> null");
const tk = tokens("#B0 t=1 seq=2 A=3/4 B=5/6 link=A1/B0 wA=10:0,10:1,10:2,10:3 raw=B0000000");
assert.strictEqual(tk.head, "B0");
assert.strictEqual(tk.t, "1");
assert.strictEqual(tk.A, "3/4");
assert.strictEqual(tk.link, "A1/B0");
assert.strictEqual(tk.raw, "B0000000");

// 8. "为什么没波形"必须说得出来（waveValid / 全 0 / 回路断开 / 没收到）
feed("#B0 t=1300 seq=0 mode=A./B. set=0/0 A=12/150 B=0/30 link=A1/B1 wA=20:40,20:50,20:60,20:70 wB=!0:0,0:0,0:0,0:0");
assert.strictEqual(scopeCards[0].waveValid, true);
assert.strictEqual(scopeCards[1].waveValid, false);
assert.ok(/输出中/.test(scopeCards[0].waveText.textContent), "A 应在输出：" + scopeCards[0].waveText.textContent);
assert.ok(/整组作废/.test(scopeCards[1].waveText.textContent), "B 应报整组作废：" + scopeCards[1].waveText.textContent);

feed("#B0 t=1400 seq=0 mode=A./B. set=0/0 A=12/150 B=0/30 link=A1/B1 wA=20:0,20:0,20:0,20:0 wB=10:5,10:5,10:5,10:5");
assert.ok(/全 0/.test(scopeCards[0].waveText.textContent), "A 全 0 要直说：" + scopeCards[0].waveText.textContent);

feed("#B0 t=1500 seq=0 mode=A./B. set=0/0 A=12/150 B=0/30 link=A0/B1 wA=20:40,20:50,20:60,20:70 wB=10:5,10:5,10:5,10:5");
assert.ok(/回路断开/.test(scopeCards[0].waveText.textContent), "A 回路断开要直说：" + scopeCards[0].waveText.textContent);

console.log("serial_scope parse tests passed");

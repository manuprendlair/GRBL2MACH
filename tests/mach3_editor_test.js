// mach3_editor_test.js - checks the core of tools/mach3_editor.html (Node.js)
//
// Loads the editor's core script (profile parsing, byte-exact saving, GRBL2MACH mapping) and checks
// it on Mach3 profiles: unchanged file rebuilt byte for byte, edited values written in Mach3's
// format with every other byte kept, and the GRBL2MACH view equal to what the bridge computes.
//
// Usage: node tests/mach3_editor_test.js [profile.xml ...]   (default: tests/mach3_test.xml)

const fs = require("fs");
const path = require("path");

const html = fs.readFileSync(path.join(__dirname, "..", "tools", "mach3_editor.html"), "utf8");
const core = html.match(/<script id="core">([\s\S]*?)<\/script>/)[1];
const mod = { exports: {} };
new Function("module", "TextDecoder", "TextEncoder", core)(mod, TextDecoder, TextEncoder);
const { parseProfile, buildFile, grbl2machView, mach3Number } = mod.exports;

let failures = 0;
const check = (cond, what) => { console.log((cond ? "  ok   : " : "  FAIL : ") + what); if (!cond) failures++; };
const files = process.argv.length > 2 ? process.argv.slice(2) : [path.join(__dirname, "mach3_test.xml")];

for (const file of files) {
  console.log("=== " + file);
  const bytes = new Uint8Array(fs.readFileSync(file));
  const p = parseProfile(bytes);
  check(p.isMach3, `read as a Mach3 profile (${p.tags.size} values)`);

  const same = buildFile(p, new Map());
  check(same.length === bytes.length && same.every((b, i) => b === bytes[i]), "saved without change: byte for byte the same file");

  const edits = new Map([["Steps0", "802."], ["Vel0", mach3Number(900 / 60)], ["XPullOff", "1.000"], ["Input22Neg", "1"]]);
  const q = parseProfile(buildFile(p, edits));
  check([...edits].every(([k, v]) => q.tags.get(k).value === v), "edited values written: " + [...edits].map(([k, v]) => `${k}=${v}`).join(" "));
  const mask = (t) => t.replace(/<(Steps0|Vel0|XPullOff|Input22Neg)>[^<]*</g, "<$1>#<");
  check(mask(q.text) === mask(p.text), "every other byte unchanged (accents, layout, other tags)");
  check(mach3Number(45) === "45." && mach3Number(13.33) === "13.33" && mach3Number(15) === "15.", "numbers in Mach3's format (45. 13.33 15.)");

  let refused = false;
  try { buildFile(p, new Map([["Profile", "é"]])); } catch (e) { refused = true; }
  check(refused, "non-ASCII value refused");

  const v = grbl2machView(p);
  const pins = v.pins.map((n) => n.toString(16).toUpperCase().padStart(2, "0")).join("");
  console.log("  GRBL2MACH view: invert 0x" + v.invert.toString(16).toUpperCase() + ", pins " + pins + ", " +
    v.settings.map((s) => `$${s.id}=${s.value}`).join(" "));
  if (/mach3_test\.xml$/.test(file)) {
    const get = (id) => v.settings.find((s) => s.id === id).value;
    check(pins === "00730073007300701200" && v.invert === 0x8f, "controller configuration as Mach3 sends it (00730073007300701200, 0x8F)");
    check(get(100) === 801.5 && get(110) === 799.8 && get(112) === 499.98 && get(132) === 50 && get(190) === 639.84 && get(27) === 0.5 && get(30) === 24000,
      "grblHAL settings as the bridge's importer computes them");
    check(v.warnings.length === 0, "no warning for the reference machine");
  }
}

console.log(failures ? `RESULT: FAIL (${failures})` : "RESULT: PASS");
process.exit(failures ? 1 : 0);

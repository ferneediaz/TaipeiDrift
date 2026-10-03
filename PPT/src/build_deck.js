// Taipei Drift pitch deck, minimal dramatic version
const pptxgen = require("pptxgenjs");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");
const fa = require("react-icons/fa");
const path = require("path");
const { applyTheme } = require(process.env.SKILL_DIR + "/scripts/apply_theme.js");

const OUT = process.argv[2] || "TaipeiDrift_Pitch.pptx";
const HERE = __dirname;

const THEME = {
  name: "Taipei Drift Night",
  headFontFace: "Arial Black",
  bodyFontFace: "Arial",
  colors: {
    dk1: "05070A", lt1: "FFFFFF", dk2: "161B22", lt2: "8B949E",
    accent1: "FFB000", // amber: sun, us
    accent2: "FF3B30", // red: drift, danger
    accent3: "2EE57A", // green: fixed
    accent4: "3A424D",
    accent5: "C9D1D9",
    accent6: "232A33",
    hlink: "FFB000", folHlink: "C98C00",
  },
};
const HEX = THEME.colors;

async function icon(Comp, color, size = 256) {
  const svg = ReactDOMServer.renderToStaticMarkup(React.createElement(Comp, { color: "#" + color, size: String(size) }));
  return "image/png;base64," + (await sharp(Buffer.from(svg)).png().toBuffer()).toString("base64");
}

(async () => {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_WIDE";
  pres.title = "Taipei Drift";
  pres.author = "Team Taipei Drift";
  pres.company = "Team Taipei Drift";
  pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
  const C = pres.SchemeColor;
  const W = 13.333, H = 7.5, M = 0.7;

  pres.defineSlideMaster({
    title: "NIGHT",
    background: { color: C.text1 },
    objects: [
      { placeholder: { options: { name: "title", type: "title", x: M, y: 0.5, w: W - 2 * M, h: 0.8, fontSize: 18, color: C.background2, bold: false, fontFace: "Arial", align: "left", valign: "middle", margin: 0, charSpacing: 4 }, text: "" } },
      { text: { text: "TAIPEI DRIFT", options: { x: M, y: H - 0.55, w: 4, h: 0.3, fontSize: 10, color: C.accent4, bold: true, charSpacing: 4, margin: 0 } } },
    ],
    slideNumber: { x: W - M - 0.6, y: H - 0.55, w: 0.6, h: 0.3, fontSize: 10, color: C.accent4, align: "right" },
  });

  const I = {
    cam: await icon(fa.FaCamera, HEX.lt1), imu: await icon(fa.FaSyncAlt, HEX.lt1), sun: await icon(fa.FaSun, HEX.dk1),
    map: await icon(fa.FaMap, HEX.lt1), target: await icon(fa.FaCrosshairs, HEX.dk1), jam: await icon(fa.FaSatelliteDish, HEX.accent2),
    play: await icon(fa.FaPlay, HEX.dk1), sunA: await icon(fa.FaSun, HEX.accent1), mapA: await icon(fa.FaMap, HEX.accent1), shieldA: await icon(fa.FaShieldAlt, HEX.accent1),
  };

  // reticle motif: thin rings + cross lines
  const reticle = (s, cx, cy, r, color, t = 60) => {
    [1, 0.62, 0.25].forEach((k, i) => s.addShape(pres.shapes.OVAL, { x: cx - r * k, y: cy - r * k, w: 2 * r * k, h: 2 * r * k, fill: { type: "none" }, line: { color, width: i === 2 ? 2 : 1, transparency: t }, objectName: "reticle ring " + i }));
    s.addShape(pres.shapes.LINE, { x: cx - r * 1.15, y: cy, w: r * 2.3, h: 0, line: { color, width: 1, transparency: t }, objectName: "reticle h" });
    s.addShape(pres.shapes.LINE, { x: cx, y: cy - r * 1.15, w: 0, h: r * 2.3, line: { color, width: 1, transparency: t }, objectName: "reticle v" });
  };
  const big = (s, text, opts) => s.addText(text, Object.assign({ fontFace: "Arial Black", bold: false, margin: 0, isTextBox: true, valign: "middle" }, opts));
  const small = (s, text, opts) => s.addText(text, Object.assign({ fontFace: "Arial", margin: 0, isTextBox: true, valign: "middle", color: C.background2 }, opts));
  const add = (sec) => pres.addSlide({ masterName: "NIGHT", sectionTitle: sec });

  // 1. Title
  pres.addSection({ title: "Opening" });
  {
    const s = add("Opening");
    reticle(s, 9.9, 3.75, 3.1, HEX.accent1, 55);
    s.addShape(pres.shapes.OVAL, { x: 9.9 - 0.12, y: 3.75 - 0.12, w: 0.24, h: 0.24, fill: { color: C.accent1 }, line: { type: "none" }, objectName: "reticle dot" });
    small(s, "EDTH TAIPEI 2026  |  CHALLENGE 2", { x: M, y: 1.6, w: 7, h: 0.4, fontSize: 14, color: C.accent1, bold: true, charSpacing: 4 });
    big(s, "TAIPEI\nDRIFT", { x: M, y: 2.1, w: 8, h: 2.9, fontSize: 88, color: C.background1, valign: "top", lineSpacingMultiple: 0.9 });
    small(s, "GNSS jammed. Still on target.", { x: M, y: 5.1, w: 8, h: 0.6, fontSize: 28, color: C.accent5 });
    s.addNotes("[0:00-0:15] A drone over Zaporizhzhia, or off Kinmen. The jammer switches on. GNSS is gone. We are Taipei Drift, and our drone keeps flying to its target anyway.");
  }

  // 2. Problem
  pres.addSection({ title: "Problem" });
  {
    const s = add("Problem");
    s.addText("THE PROBLEM", { placeholder: "title" });
    big(s, "36 s", { x: M, y: 1.3, w: 8.5, h: 3.6, fontSize: 220, color: C.accent2 });
    small(s, "GNSS jammed  →  50 m off target", { x: M + 0.1, y: 5.0, w: 9, h: 0.7, fontSize: 30, color: C.background1 });
    s.addImage({ data: I.jam, x: 10.3, y: 2.0, w: 2.2, h: 2.2, objectName: "jamming icon" });
    small(s, "After 4 km: 3 km lost", { x: 9.4, y: 4.5, w: 3.3, h: 0.5, fontSize: 18, color: C.background2, align: "center" });
    s.addNotes("[0:15-0:35] Low-cost drone, GNSS jammed: after 36 seconds the IMU alone is 50 metres off; after 4 km it is 3 km off (median of 30 Mid-Air flights; our Wufeng simulation). Jamming is routine in Ukraine and around Taiwan's islands. Existing fixes like Maxar Raptor need a GPU and licensed 3D data, built for aircraft budgets, not 500-dollar drones.");
  }

  // 3. Solution
  pres.addSection({ title: "Solution" });
  {
    const s = add("Solution");
    s.addText("OUR SOLUTION", { placeholder: "title" });
    const items = [[I.cam, "CAMERA", C.accent6], [I.imu, "IMU", C.accent6], [I.sun, "SUN", C.accent1], [I.map, "FREE MAP", C.accent6]];
    const d = 1.75, gap = 0.55, x0 = M + 0.1;
    items.forEach(([img, label, fill], i) => {
      const x = x0 + i * (d + gap);
      s.addShape(pres.shapes.OVAL, { x, y: 2.0, w: d, h: d, fill: { color: fill }, line: { type: "none" }, objectName: label + " circle" });
      s.addImage({ data: img, x: x + 0.45, y: 2.45, w: d - 0.9, h: d - 0.9, objectName: label + " icon" });
      small(s, label, { x: x - 0.3, y: 3.95, w: d + 0.6, h: 0.5, fontSize: 18, bold: true, color: C.background1, align: "center", charSpacing: 3 });
      if (i < 3) small(s, "+", { x: x + d, y: 2.0, w: gap, h: d, fontSize: 36, color: C.background2, align: "center" });
    });
    const xe = x0 + 4 * (d + gap) - gap;
    small(s, "=", { x: xe, y: 2.0, w: gap + 0.1, h: d, fontSize: 40, color: C.accent1, align: "center", bold: true });
    s.addShape(pres.shapes.OVAL, { x: xe + gap + 0.15, y: 1.85, w: 2.05, h: 2.05, fill: { color: C.accent3 }, line: { type: "none" }, objectName: "position circle" });
    s.addImage({ data: I.target, x: xe + gap + 0.6, y: 2.3, w: 1.15, h: 1.15, objectName: "position icon" });
    small(s, "POSITION", { x: xe + gap - 0.1, y: 3.95, w: 2.55, h: 0.5, fontSize: 18, bold: true, color: C.accent3, align: "center", charSpacing: 3 });
    // three claims
    const claims = ["NO GPU", "NO LICENCE", "+35 g"];
    claims.forEach((t, i) => big(s, t, { x: M + i * 4.0, y: 5.15, w: 3.8, h: 0.9, fontSize: 36, color: i === 2 ? C.accent1 : C.background1 }));
    s.addNotes("[0:35-0:55] The drone already has a camera and an IMU. We add a few-dollar sun sensor, a compass nobody can jam, and free aerial photos. One Kalman filter fuses it all; every 300 m the camera fixes the position on the map, and a statistical gate refuses fixes that don't fit. No GPU, no data licence, 35 grams.");
  }

  // 4. Proof: chart
  pres.addSection({ title: "Proof" });
  {
    const s = add("Proof");
    s.addText("DOES IT WORK?", { placeholder: "title" });
    s.addChart(pres.charts.BAR, [{ name: "Median error", labels: ["IMU", "+ SUN", "+ CAMERA", "+ MAP"], values: [3000, 2400, 57, 18] }], {
      x: M, y: 1.4, w: 7.6, h: 5.2, barDir: "col", barGapWidthPct: 35,
      chartColors: [HEX.accent2, HEX.accent2, HEX.accent1, HEX.accent3],
      valAxisLogScaleBase: 10, valAxisMinVal: 1, valAxisMaxVal: 10000, valAxisHidden: true,
      valGridLine: { style: "none" }, catGridLine: { style: "none" }, catAxisLineShow: false,
      catAxisLabelColor: HEX.lt2, catAxisLabelFontSize: 16, catAxisLabelFontFace: "+mn-lt", catAxisLabelFontBold: true,
      showValue: true, dataLabelPosition: "outEnd", dataLabelFormatCode: '0" m"', dataLabelColor: HEX.lt1, dataLabelFontSize: 22, dataLabelFontBold: true, dataLabelFontFace: "+mn-lt",
      showLegend: false, objectName: "error by sensor chart",
    });
    big(s, "3 km", { x: 8.9, y: 1.7, w: 3.8, h: 1.3, fontSize: 80, color: C.accent2, align: "right" });
    small(s, "↓", { x: 8.9, y: 3.0, w: 3.8, h: 0.8, fontSize: 48, color: C.background2, align: "right" });
    big(s, "18 m", { x: 8.9, y: 3.8, w: 3.8, h: 1.3, fontSize: 80, color: C.accent3, align: "right" });
    small(s, "median error, 4.3 km without GNSS", { x: 8.9, y: 5.3, w: 3.8, h: 0.5, fontSize: 14, align: "right" });
    s.addNotes("[0:55-1:15] Each source we add cuts the error: IMU alone 3 km; the sun fixes the heading; the camera fixes the speed; the map fixes bring it to 18 m. Simulated 100 m flight over real Wufeng aerial imagery, with a map two years older than the ground. On sealed flights we never tuned on: 16 to 36 m, no wrong fix.");
  }

  // 5. Real flight
  {
    const s = add("Proof");
    s.addText("REAL FLIGHT  |  TAIWAN  |  NO GNSS", { placeholder: "title" });
    big(s, "3.3 m", { x: M, y: 1.3, w: W - 2 * M, h: 3.3, fontSize: 200, color: C.accent3 });
    const facts = [["4 km", "flown blind"], ["0 / 1,646", "wrong fixes"], ["20 / 20", "runs locked"]];
    facts.forEach(([n, l], i) => {
      const x = M + i * 4.05;
      big(s, n, { x, y: 4.85, w: 3.9, h: 0.9, fontSize: 40, color: C.background1 });
      small(s, l, { x, y: 5.75, w: 3.9, h: 0.45, fontSize: 18 });
    });
    s.addNotes("[1:15-1:35] And on a real flight: real drone photos over the Tuniu River in Miaoli, RTK as truth. 4 km without GNSS: 3.3 m median error. Dead reckoning alone: 54 m. 1,646 map fixes accepted, zero wrong. Lock kept in 20 of 20 runs. (Barometer simulated.)");
  }

  // 6. Innovation
  pres.addSection({ title: "Innovation" });
  {
    const s = add("Innovation");
    s.addText("WHAT OTHERS MISSED", { placeholder: "title" });
    const cols = [
      [I.sunA, "1.3°", "heading from the sun", "unjammable"],
      [I.mapA, "$0", "maps: free, 2 years old", "no vendor lock-in"],
      [I.shieldA, "0", "wrong fixes accepted", "knows when it's wrong"],
    ];
    const cw = (W - 2 * M) / 3;
    cols.forEach(([img, n, l1, l2], i) => {
      const x = M + i * cw;
      s.addImage({ data: img, x: x + 0.05, y: 1.7, w: 0.9, h: 0.9, objectName: "innovation icon " + i });
      big(s, n, { x, y: 2.85, w: cw - 0.3, h: 1.8, fontSize: 110, color: C.background1 });
      small(s, l1, { x, y: 4.8, w: cw - 0.3, h: 0.5, fontSize: 20, color: C.accent5 });
      small(s, l2.toUpperCase(), { x, y: 5.35, w: cw - 0.3, h: 0.45, fontSize: 14, color: C.accent1, bold: true, charSpacing: 3 });
    });
    s.addNotes("[1:35-1:55] Three insights. The sun is a compass nobody can jam: heading error drops from 55 to 1.3 degrees. A free photo two years old is enough: no vendor 3D data. And it knows when it is wrong: zero wrong fixes accepted, the stated error bound held 100 % on sealed flights. Night: a thermal camera against the same maps, same filter; that is our next step.");
  }

  // 7. Market & business
  pres.addSection({ title: "Market" });
  {
    const s = add("Market");
    s.addText("THE MARKET", { placeholder: "title" });
    const cols = [["48,750", "drones, Taiwan, 2026–27"], ["4 M", "drones a year, Ukraine"], ["$22.8 B", "military drones, 2030"]];
    const cw = (W - 2 * M) / 3;
    cols.forEach(([n, l], i) => {
      const x = M + i * cw;
      big(s, n, { x, y: 1.7, w: cw - 0.2, h: 1.6, fontSize: 52, color: C.background1 });
      small(s, l, { x, y: 3.3, w: cw - 0.2, h: 0.5, fontSize: 18 });
    });
    s.addShape(pres.shapes.LINE, { x: M, y: 4.35, w: W - 2 * M, h: 0, line: { color: HEX.accent4, width: 1 }, objectName: "divider" });
    big(s, "$200", { x: M, y: 4.6, w: 4.2, h: 1.6, fontSize: 96, color: C.accent1 });
    small(s, "licence per drone,\nsold to drone makers", { x: M + 4.3, y: 4.75, w: 5.5, h: 1.3, fontSize: 24, color: C.background1 });
    small(s, "Sources: Taiwan MND 2025; Kyiv Post / Bloomberg 2025; MarketsandMarkets 2025", { x: M, y: 6.45, w: 9, h: 0.3, fontSize: 10, color: C.accent4 });
    s.addNotes("[1:55-2:20] Taiwan buys 48,750 drones in 2026-27, a 1.56-billion-dollar programme, built in Taiwan with no Chinese parts: open maps and local software fit that rule. Ukraine builds millions a year. Military drones: 22.8 billion dollars by 2030. We license per drone to the makers, about 200 dollars, plus map packs. One million drones is 200 million dollars a year.");
  }

  // 8. Team + demo
  pres.addSection({ title: "Team" });
  {
    const s = add("Team");
    s.addText("THE TEAM", { placeholder: "title" });
    const team = [["DUSTIN", "navigator"], ["ALESSANDRO", "fusion filter"], ["DAN", "simulator"], ["ILHAN", "real flight"], ["FELIX", "terrain nav"]];
    const cw = (W - 2 * M) / team.length;
    team.forEach(([n, r], i) => {
      const x = M + i * cw;
      s.addShape(pres.shapes.OVAL, { x: x + (cw - 1.5) / 2, y: 1.9, w: 1.5, h: 1.5, fill: { color: C.accent6 }, line: { color: i % 2 ? HEX.accent4 : HEX.accent1, width: 2 }, objectName: n + " avatar" });
      big(s, n[0], { x: x + (cw - 1.5) / 2, y: 1.9, w: 1.5, h: 1.5, fontSize: 48, color: C.background1, align: "center" });
      small(s, n, { x, y: 3.65, w: cw, h: 0.5, fontSize: 18, bold: true, color: C.background1, align: "center", charSpacing: 2 });
      small(s, r, { x, y: 4.1, w: cw, h: 0.4, fontSize: 16, align: "center" });
    });
    big(s, "48 h.  304 tests.  Sealed test, run once.", { x: M, y: 5.1, w: W - 2 * M, h: 0.9, fontSize: 30, color: C.accent1, align: "center" });
    s.addNotes("[2:20-2:35] Five people, every layer built by us: navigator, fusion filter, simulator over Taiwan, real-flight test, terrain navigation. We froze the code before testing on flights we had never seen. That is why you can trust the numbers. [Add one-word backgrounds if you want.]");
  }

  // 9. Demo
  pres.addSection({ title: "Demo" });
  {
    const s = add("Demo");
    s.addText("DEMO", { placeholder: "title" });
    const fw = 3.4, fh = fw * (802 / 648);
    s.addImage({ path: path.join(HERE, "wufeng_map.png"), x: M, y: 1.35, w: fw * 0.88, h: fh * 0.88, objectName: "Wufeng flights" });
    const cx = 6.65, cy = 3.7;
    s.addShape(pres.shapes.OVAL, { x: cx - 1.3, y: cy - 1.3, w: 2.6, h: 2.6, fill: { color: C.accent1 }, line: { type: "none" }, objectName: "play circle" });
    s.addImage({ data: I.play, x: cx - 0.45, y: cy - 0.6, w: 1.2, h: 1.2, objectName: "play icon" });
    small(s, "VIDEO LINK", { x: cx - 2, y: cy + 1.5, w: 4, h: 0.5, fontSize: 16, bold: true, color: C.accent1, align: "center", charSpacing: 4, hyperlink: { url: "https://example.com/replace-with-demo-video" } });
    const ih = 5.0, iw = ih * (800 / 1252);
    s.addImage({ path: path.join(HERE, "floor.jpg"), x: W - M - iw, y: 1.3, w: iw, h: ih, objectName: "floor mosaic" });
    small(s, "Simulated flights, Taichung", { x: M, y: 6.45, w: 3.5, h: 0.3, fontSize: 12 });
    small(s, "Real camera, path from floor texture", { x: W - M - 4, y: 6.45, w: 4, h: 0.3, fontSize: 12, align: "right" });
    s.addNotes("[2:35-2:48] Play the video or point at it: GNSS cut mid-flight over Taichung, the estimate stays on the track. Right: a real camera on a cart finds its own path from the floor alone; the loop closes to 0.1 mm. [Replace the VIDEO LINK hyperlink with the real video URL before upload.]");
  }

  // 10. CTA
  pres.addSection({ title: "Close" });
  {
    const s = add("Close");
    reticle(s, 6.67, 3.5, 2.9, HEX.accent1, 75);
    big(s, "FLY IT\nWITH US.", { x: M, y: 1.4, w: W - 2 * M, h: 3.2, fontSize: 80, color: C.background1, align: "center", lineSpacingMultiple: 0.9 });
    small(s, "DRONE MAKERS   |   UNITS   |   INVESTORS", { x: M, y: 5.0, w: W - 2 * M, h: 0.6, fontSize: 24, bold: true, color: C.accent1, align: "center", charSpacing: 4 });
    small(s, "Talk to us right after the pitch", { x: M, y: 5.6, w: W - 2 * M, h: 0.5, fontSize: 18, align: "center" });
    s.addNotes("[2:48-3:00] Drone makers: lend us an airframe and one flight day. Units in Ukraine and Taiwan: tell us your night and over-water missions. Investors: talk to us right after this. Taipei Drift: GNSS jammed, still on target. Thank you.");
  }

  await pres.writeFile({ fileName: OUT });
  await applyTheme(OUT, THEME);
  console.log("wrote", OUT);
})().catch((e) => { console.error(e); process.exit(1); });

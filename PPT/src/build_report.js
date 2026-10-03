// Sensor cost report (Word) for Team Taipei Drift
const fs = require("fs");
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, WidthType, ShadingType,
  HeadingLevel, AlignmentType, BorderStyle, LevelFormat, Footer, PageNumber, ExternalHyperlink,
  TableOfContents,
} = require("docx");

const OUT = process.argv[2];
const FONT = "Arial";
const W = 9026; // A4 text width in DXA with 1" margins

// ---------- helpers ----------
const runs = (s, base = {}) => {
  // **bold** and [n] citations stay plain text; support **bold** only
  const out = [];
  s.split(/(\*\*[^*]+\*\*)/).forEach((part) => {
    if (!part) return;
    if (part.startsWith("**")) out.push(new TextRun({ text: part.slice(2, -2), bold: true, ...base }));
    else out.push(new TextRun({ text: part, ...base }));
  });
  return out;
};
const P = (s, opts = {}) => new Paragraph({ children: runs(s, opts.run || {}), spacing: { after: 120, line: 276 }, ...opts.para });
const H1 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun(s)], spacing: { before: 360, after: 160 } });
const H2 = (s) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun(s)], spacing: { before: 240, after: 120 } });
const B = (s) => new Paragraph({ numbering: { reference: "bullets", level: 0 }, children: runs(s), spacing: { after: 80, line: 276 } });
const N = (s) => new Paragraph({ numbering: { reference: "numbers", level: 0 }, children: runs(s), spacing: { after: 80, line: 276 } });
const caption = (s) => new Paragraph({ children: [new TextRun({ text: s, italics: true, size: 18, color: "555555" })], spacing: { before: 60, after: 200 } });

const border = { style: BorderStyle.SINGLE, size: 4, color: "BFBFBF" };
const borders = { top: border, bottom: border, left: border, right: border };
function table(header, rows, widths, opts = {}) {
  const total = widths.reduce((a, b) => a + b, 0);
  const cell = (t, w, head, shade) => new TableCell({
    borders, width: { size: w, type: WidthType.DXA },
    shading: head ? { fill: "1F2D3D", type: ShadingType.CLEAR, color: "auto" } : shade ? { fill: shade, type: ShadingType.CLEAR, color: "auto" } : undefined,
    margins: { top: 60, bottom: 60, left: 100, right: 100 },
    children: String(t).split("\n").map((line) => new Paragraph({ children: runs(line, { size: 17, color: head ? "FFFFFF" : "000000", bold: head || undefined }) })),
  });
  return new Table({
    width: { size: total, type: WidthType.DXA }, columnWidths: widths,
    rows: [
      new TableRow({ tableHeader: true, children: header.map((h, i) => cell(h, widths[i], true)) }),
      ...rows.map((r, ri) => new TableRow({ children: r.map((c, i) => cell(c, widths[i], false, (opts.highlight || []).includes(ri) ? "FFF4D6" : ri % 2 ? "F5F7F9" : undefined)) })),
    ],
  });
}

// ---------- references ----------
const REFS = [
  /* 1 */ ["Xie, N., Theuwissen, A.J.P., Büttgen, B., Hakkesteegt, H., Jansen, H., Leijtens, J. (2010). Micro-Digital Sun Sensor: an imaging sensor for space applications. Proc. IEEE Int. Symp. Industrial Electronics (ISIE), 4–7 July 2010. TU Delft Research Portal.", "https://research.tudelft.nl/en/publications/micro-digital-sun-sensor-an-imagining-sensor-for-space-applicatio"],
  /* 2 */ ["Leijtens, J., Theuwissen, A., Rao, P.R., Wang, X., Xie, N. (2007). Active pixel sensors: the sensor of choice for future space applications. Proc. SPIE 6744, 67440V. doi:10.1117/12.735512 (TU Delft repository copy).", "https://repository.tudelft.nl/file/File_1dd4cc44-0421-4331-943f-35815ba1be95"],
  /* 3 */ ["de Boer, B.M., Durkut, M., Laan, E., Hakkesteegt, H., Theuwissen, A., Xie, N., Leijtens, J.L., Urquijo, E., Bruins, P. (2012). MiniDSS: a low-power and high-precision miniaturized digital sun sensor. TNO publication.", "https://publications.tno.nl/publication/100130/q3tjfC/deboer-2012-minidss.pdf"],
  /* 4 */ ["Fan, Peng and Gao (2016). Digital sun sensor with a V-shaped slit over a linear detector. Rev. Sci. Instrum. 87, 075003. doi:10.1063/1.4958696 (paywalled; abstract and the summary in MDPI Sensors 21(4):1472 used: 65° FOV in both axes, 0.1°).", "https://doi.org/10.1063/1.4958696"],
  /* 5 */ ["MDPI Sensors (2021) 21(4):1472. Improved accuracy of a single-slit digital sun sensor design for CubeSat application using sub-pixel interpolation (review of slit/linear-array sun sensors incl. Fan et al.).", "https://www.mdpi.com/1424-8220/21/4/1472"],
  /* 6 */ ["CubeSatShop. Nano-SSOC-A60 analog sun sensor (Solar MEMS), €2,500, 3.7 g.", "https://www.cubesatshop.com/product/nano-ssoc-a60-analog-sun-sensor/"],
  /* 7 */ ["Lens R&D. Product and price catalogue 2018 (BiSon64 family), via CubeSatShop.", "https://www.cubesatshop.com/wp-content/uploads/2017/01/Lens-RD-Product-and-Price-catalogue-2018.pdf"],
  /* 8 */ ["CubeSatShop. NSS Fine Sun Sensor (NewSpace Systems NFSS-411), $12,000, 0.1° RMS, 140° FOV.", "https://www.cubesatshop.com/product/digital-fine-sun-sensor/"],
  /* 9 */ ["Price listings for ams-OSRAM TSL1401CL 128×1 linear sensor array (Future Electronics; FindChips aggregate).", "https://findchips.com/search/tsl1401c?f=dsa"],
  /* 10 */ ["Flywing-tech. TSL1401CL tiered pricing ($8.06 at 1 pc to $5.99 at 1,000 pcs).", "https://www.flywing-tech.com/product-detail/specialized-sensors-ams-tsl1401cl-5c27a318"],
  /* 11 */ ["RS Components (ES). Hamamatsu S11639-01 CMOS linear image sensor, €231.67 (1–4 pcs), €224.72 (5+).", "https://es.rs-online.com/web/p/fotodiodos/0893858"],
  /* 12 */ ["Tom's Hardware (2022). Raspberry Pi offers RP2040 bulk purchase from $0.70 per chip.", "https://tomshardware.com/news/raspberry-pi-offers-rp2040-direct"],
  /* 13 */ ["The Pi Hut. OV9281 global shutter camera module for Raspberry Pi (≈£25).", "https://thepihut.com/products/ov9281-global-shutter-camera-module-for-raspberry-pi-4b-5-1mp-fov-79"],
  /* 14 */ ["The Pi Hut. Raspberry Pi Camera Module 3 ($25 standard, $35 wide, list price).", "https://thepihut.com/products/raspberry-pi-camera-module-3"],
  /* 15 */ ["Digi-Key. TE Connectivity MS561101BA03-50 barometric pressure sensor, $12.32 (1 pc) to $5.78 (reel of 3,600).", "https://www.digikey.com/en/products/detail/mikroelektronika/MIKROE-4903/16182509"],
  /* 16 */ ["TE Connectivity / MEAS. MS5611-01BA03 datasheet: resolution RMS 0.012 mbar at OSR 4096, altitude resolution 10 cm.", "https://www.embeddedadventures.com/datasheets/ms5611.pdf"],
  /* 17 */ ["Bosch Sensortec. BMP390 product page and flyer: relative accuracy ±3 Pa (±25 cm), RMS noise 0.02 Pa (lowest bandwidth).", "https://bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390"],
  /* 18 */ ["Cytech Systems. BMP390 component pricing (from $5.60).", "https://www.cytechsystems.com/product/bmp390"],
  /* 19 */ ["Adafruit. DPS310 precision barometric pressure sensor breakout (±0.002 hPa precision, ±1 hPa absolute), $6.95 (1–9), $5.56 (100+).", "https://www.adafruit.com/products/4494"],
  /* 20 */ ["PX4 User Guide. Holybro Pixhawk 6C: on-board sensors ICM-42688-P and BMI055 (IMU), MS5611 (barometer), IST8310 (magnetometer).", "https://docs.px4.io/main/en/flight_controller/pixhawk6c.html"],
  /* 21 */ ["Holybro Store. Pixhawk 6C flight controller, $211.99.", "https://holybro.com/products/pixhawk-6c"],
  /* 22 */ ["robu.in / Switch Science. Benewake TF02-Pro (40 m, 50 g); EU retail €94–120.", "https://stgaws.robu.in/?p=715933"],
  /* 23 */ ["Acroname. LightWare price list: LW20/C $279, SF30/C $299, SF30/D $399 (200 m, 36 g), SF45 $449.", "https://www.acroname.com/store-grid/field_category/lidar/field_manufacturer/lightware"],
  /* 24 */ ["Digi-Key product highlight. LightWare LW20/C microLiDAR: 0.2–100 m, ±5 cm (< 500 readings/s), 48–5,000 readings/s, 19 g, IP67, first/last pulse.", "https://www.digikey.com/en/product-highlight/l/lightware-lidar/lw20-c-microlidar-distance-sensor"],
  /* 25 */ ["RobotShop / SparkFun. Benewake TF03-180 (100 m operating, 180 m max), $249.90–$274.95.", "https://www.robotshop.com/products/benewake-tf03-lidar-led-rangefinder-ip67-180-m"],
  /* 26 */ ["LIDAR Magazine (2025). LightWare's all-new GRF-500: 0.2–500 m, 10.7 g, ~0.6 W, 905 nm Class 1M.", "https://lidarmag.com/2025/03/31/lightwares-all-new-grf-500-ranges-further-for-eo-ir-payloads/"],
  /* 27 */ ["MYBOTSHOP. LightWare GRF-500 LiDAR, €524.95 incl. 19 % VAT.", "https://www.mybotshop.de/Lightware-GRF-500-Lidar_1"],
  /* 28 */ ["Livox. Mid-360 specifications: 40 m at 10 % reflectivity, 265 g.", "https://www.livoxtech.com/mid-360/specs"],
  /* 29 */ ["rcdrone.top. Livox Mid-360 listing, $1,091.80.", "https://rcdrone.top/products/livox-mid-360-lidar"],
  /* 30 */ ["Livox. Avia product page: up to 450 m detection range, 498 g, triple return.", "https://www.livoxtech.com/avia"],
  /* 31 */ ["C.R. Kennedy (AU) and US retail listings for Livox Avia: AUD 4,290; $2,079.", "https://survey.crkennedy.com.au/products/livoxavia/livox-avia-lidar"],
  /* 32 */ ["Geo Week News (2019). Ouster OS1-32 pricing ($8,000 commercial list); eBay listing OS1-32: $5,770.", "https://www.geoweeknews.com/news/ouster-64-channel-lidar-1-6-cost"],
  /* 33 */ ["Jenoptik. DLEM laser rangefinder modules datasheet (DLEM 20: up to 5 km, < 33 g, ±0.5 m). Price not published.", "https://www.jenoptik.com/-/media/websitedocuments/optics/sensor/dlem_datasheet_20230818.pdf"],
  /* 34 */ ["HWBusters (2026). NVIDIA Jetson prices jump; Orin Nano Super Developer Kit now $399 (was $249).", "https://hwbusters.com/news/nvidia-jetson-prices-jump-up-to-101-the-249-orin-nano-super-is-now-399/"],
  /* 35 */ ["Notebookcheck (2026). Raspberry Pi 5 now costs up to $205 due to RAM crisis (4 GB: $85).", "https://www.notebookcheck.net/Raspberry-Pi-5-now-costs-up-to-205-due-to-RAM-crisis.1218213.0.html"],
  /* 36 */ ["PetaPixel (2023). Raspberry Pi's new Global Shutter Camera (Sony IMX296, 1456×1088) costs $50.", "https://petapixel.com/2023/03/09/raspberry-pis-new-global-shutter-camera-costs-just-50/"],
  /* 37 */ ["Gerhard, S., Tokekar, P. (2020). Experimental evaluation of a pseudo-Doppler direction-finding system for localizing radio tags. arXiv:2003.00386 (four antennas, RF switch, HackRF One + Opera Cake, 150 MHz, on a four-rotor UAS).", "https://arxiv.org/pdf/2003.00386"],
  /* 38 */ ["RTL-SDR.com. HackRF Opera Cake released: a rapid RF switching board (US$190).", "https://www.rtl-sdr.com/hackrf-opera-cake-released-a-rapid-rf-switching-board/"],
  /* 39 */ ["Lab401. Opera Cake for HackRF (€189) and HackRF One (€305) listings.", "https://lab401.com/en-de/products/opera-cake-for-hackrf"],
  /* 40 */ ["SparkFun. RTL-SDR Blog V4 USB dongle with dipole antenna kit, $84.95.", "https://www.sparkfun.com/rtl-sdr-blog-v4-usb-dongle-with-dipole-antenna-kit.html"],
  /* 41 */ ["CNX Software (2023). RTL-SDR Blog V4 launched: $29.95, or $39.95 with antenna set.", "https://www.cnx-software.com/2023/08/17/rtl-sdr-blog-v4-dongle-launched-with-rafeal-r828d-tuner-chip/"],
  /* 42 */ ["KrakenRF. KrakenSDR five-channel coherent receiver for direction finding, $499 (EU €799).", "https://www.krakenrf.com/product-page/krakensdr"],
  /* 43 */ ["Digi-Key. Analog Devices ADG904BCPZ SP4T switch (2.5 GHz), $5.88 (1 pc), $3.30 (2,652+).", "https://www.digikey.com/en/products/detail/analog-devices-inc/ADG904BCPZ/1644854"],
  /* 44 */ ["BridgeCom Systems. Nagoya NA-701 VHF/UHF whip, 22 cm, SMA, $14.99 (sale) / $24.99.", "https://www.bridgecomsystems.com/products/nagoya-na-701-8-inch-whip-vhf-uhf-144-430mhz-antenna-sma-female"],
  /* 45 */ ["Moonraker. Shakespeare HA156C AIS helical stub antenna, 0.15 m, $85.49 (US).", "https://moonrakeronline.com/us/shakespeare-ha156c-ais-unity-gain-0-15m-helical-ais-antenna-integral-aluminium-bracket-20m-rg58-pl259"],
  /* 46 */ ["Taipei Drift repository, branch integration at commit d8660c9 (read only): sim/models/midair_quad/model.sdf; sim/launch/sim.launch.py; sim/config/sensor_noise.yaml; sim/config/rf.yaml; sim/RF_README.md; baseline/configs/sim_navigator.yaml; baseline/src/sensors/heading.py and sun_sensor.py; scripts/fused_replay.py; TRN/configs/laser.yaml; docs/simulation-results.md; docs/PLAN.md.", "https://github.com/dwn97/TaipeiDrift/tree/integration"],
];
const refParas = REFS.map(([t, url], i) => new Paragraph({
  spacing: { after: 80 }, indent: { left: 567, hanging: 567 },
  children: [new TextRun({ text: `[${i + 1}]\t${t} `, size: 18 }), new ExternalHyperlink({ link: url, children: [new TextRun({ text: url, style: "Hyperlink", size: 18 })] }), new TextRun({ text: " (accessed 3 Oct 2026)", size: 18 })],
}));

// ---------- content ----------
const body = [];
body.push(new Paragraph({ spacing: { before: 1200, after: 120 }, children: [new TextRun({ text: "Sensor Cost Report", bold: true, size: 52, font: FONT, color: "1F2D3D" })] }));
body.push(new Paragraph({ spacing: { after: 240 }, children: [new TextRun({ text: "The final sensor suite of our GNSS-free drone navigator: what it costs, part by part", size: 30, color: "444444" })] }));
body.push(P("Team Taipei Drift  |  EDTH Taiwan Defense Tech Hackathon 2026, Challenge 2"));
body.push(P("Version 1.1, 3 October 2026, 22:45. Updated to the final sensor specifications on branch integration, commit d8660c9 [46]. Prices retrieved on 3 October 2026 from public distributor and vendor listings."));
body.push(new Paragraph({ spacing: { before: 240, after: 120 }, border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: "FFB000", space: 1 } }, children: [] }));

// ===== Summary
body.push(H1("Summary"));
body.push(P("This report prices every sensor of our final simulated drone, as specified on the integration branch [46], and compares the optional range finder with LiDAR alternatives. For each sensor the requirement is taken from our simulation's sensor model, matched against commercial parts, and priced from public listings with sources. All prices are single-unit or small-volume list prices, not quotes."));
body.push(table(
  ["Sensor (final version)", "Specification in our simulator", "Matching part", "List price", "Added cost per drone"],
  [
    ["Down + forward camera", "2 × 1024×1024 px, 90° FOV, 25 Hz", "Platform camera; reference: Raspberry Pi Global Shutter Camera", "$50 each [36]; Camera Module 3 wide $35 [14]", "**$0** if on the drone; else ≈ $70–100"],
    ["IMU", "100 Hz, consumer MEMS (Mid-Air noise model)", "ICM-42688-P on Pixhawk 6C", "flight controller $211.99 [20][21]", "**$0** (on the flight controller)"],
    ["Barometer", "50 Hz, 10 Pa noise (≈ 0.8 m), drift 0.65 Pa/√s", "MS5611 on Pixhawk 6C", "chip $5.78–12.32 [15]", "**$0** (on the flight controller)"],
    ["Sun sensor", "0.1°, ±65° FOV (Fan et al. 2016)", "Self-built: linear array behind a V-slit + RP2040", "parts $6–9 + mechanics (estimate) [9][10][12]", "**≈ $10–20**"],
    ["Downward range finder (optional, metric flow)", "1 beam, 0.2–100 m, σ 0.02 m, 25 Hz", "LightWare LW20/C (0.2–100 m, ±5 cm, 19 g)", "$279 [23][24]", "**≈ $250–300**"],
    ["RF direction finder (over water)", "AIS receiver; 4 × 17 cm VHF stubs, RF switch at 1 kHz, one radio, −110 dBm", "Low-cost: RTL-SDR V4 + ADG904 switch + 4 whips. Reference: HackRF One + Opera Cake", "≈ $110–210 low-cost; ≈ €558 (≈ $615) reference [37–44]", "**≈ $110–210**"],
  ],
  [1500, 1950, 2050, 1800, 1726], { highlight: [3] }
));
body.push(caption("Table 1. The final sensor suite and its cost. GNSS (1 Hz, used only before the jam) is part of every drone and not costed."));
body.push(P("**Main findings.**"));
body.push(B("**The core navigator adds about $10–20 of hardware.** Cameras, IMU and barometer of the final version are standard on a drone with a Pixhawk-class flight controller [20]. The only new part for camera, sun and map navigation is the sun sensor."));
body.push(B("**The full final suite adds about $370–530** (sun sensor, LW20/C range finder and a low-cost AIS direction finder), or about $875–935 with the reference radio hardware of the paper the design follows [37]. Both the range finder and the radio are optional modules: metric flow is off by default, and the radio only helps where AIS ships are in reach."));
body.push(B("**The final range finder specification matches an existing part one-to-one.** The simulator's 0.2–100 m single beam is the LightWare LW20/C's range, at $279, 19 g, ±5 cm [24]."));
body.push(B("**A space-grade sun sensor buys nothing.** The drone's 1° tilt error, not the sensor, limits the heading (section 3.1). Space units cost €2,500 to $12,000 [6][7][8]."));
body.push(B("**LiDAR navigation is in another price class.** A scanning LiDAR that sees the ground from our flight heights costs about $2,500 with its computer and weighs about 0.5 kg [30][31][34]."));

// ===== 1 Scope
body.push(H1("1. Scope and method"));
body.push(H2("1.1 The final sensor suite"));
body.push(P("The final simulated drone is a Mid-Air-like quadcopter (1 kg, AirSim default airframe) defined in sim/models/midair_quad/model.sdf and assembled by sim/launch/sim.launch.py; the heading sensor and the camera navigator are configured in baseline/configs/sim_navigator.yaml and fused in scripts/fused_replay.py [46]. Table 2 lists every sensor as specified there."));
body.push(table(
  ["Sensor", "Specification (integration, d8660c9)", "Used by", "Where defined"],
  [
    ["Down camera", "1024×1024 px (launch default cam_res), 90° FOV, 25 Hz, pinhole; recordings used 512 px at 5 Hz", "Camera navigator, map fixes, ground speed", "model.sdf, sim.launch.py"],
    ["Forward camera", "1024×1024 px, 90° FOV, 25 Hz", "ESKF vision rotation and direction updates", "model.sdf"],
    ["IMU", "100 Hz; noise added by sensor_noise.py: gyro white 0.0005–0.005 rad/s, bias 0.001–0.01 rad/s; accel white 0.005–0.05 m/s², bias 0.01–0.1 m/s²", "ESKF prediction", "sensor_noise.yaml"],
    ["Barometer", "50 Hz, 10 Pa white noise (≈ 0.8 m, \"MS5611-class\"), drift 0.65 Pa/√s", "ESKF height", "model.sdf, sensor_noise.yaml"],
    ["GNSS", "1 Hz, ≈ 1.5 m horizontal, 3 m vertical; cut 20 s after the first fix", "Start only", "model.sdf, sim.launch.py"],
    ["Downward range finder", "Single beam (gpu_lidar), 0.20–100 m, resolution 0.01 m, σ 0.02 m, 25 Hz, co-located with the down camera", "Metric optical flow (metric_flow:=true, off by default)", "model.sdf, sim.launch.py"],
    ["Sun sensor", "Digital, Fan et al. 2016: 0.1°, ±65° FOV, mounting error per flight (heading.source: sun_digital)", "Navigator heading; fused filter yaw", "sim_navigator.yaml, sun_sensor.py"],
    ["RF direction finder", "AIS (161.975/162.025 MHz) receiver, sensitivity −110 dBm, NF 6 dB; pseudo-Doppler array: 4 helical stubs, 17 cm, on the arms at 9.5 cm radius, switched at 1 kHz into one radio; bearing σ 3° floor + 1° mounting bias", "RF position fix over water (strait world)", "rf.yaml, RF_README.md"],
  ],
  [1500, 3750, 2100, 1676]
));
body.push(caption("Table 2. Final sensor suite, as specified on branch integration [46]."));
body.push(H2("1.2 How prices were collected"));
body.push(B("Public list prices from distributors and vendors, retrieved on 3 October 2026. Where several listings exist, the range is given; quantity breaks are quoted where the source gives them."));
body.push(B("Prices are in the currency of the source. For the per-drone totals we convert at approximately 1 EUR ≈ 1.10 USD, 1 GBP ≈ 1.30 USD and 1 AUD ≈ 0.65 USD. These rates are our assumption for orientation only."));
body.push(B("Items without a public source (slit masks, filters, housings, small PCBs, cables, assembly) are marked as **estimate** and kept separate from sourced prices."));
body.push(B("Not included: export-control and qualification cost, integration labour, taxes and shipping (except where a listing includes VAT, which is stated)."));

// ===== 2 Platform sensors
body.push(H1("2. Sensors already on the platform"));
body.push(H2("2.1 IMU and barometer"));
body.push(P("A Pixhawk 6C flight controller ($211.99 [21]) carries an ICM-42688-P and a BMI055 IMU and an MS5611 barometer [20], the barometer class our simulator names. Any drone flown with such a controller therefore needs no extra IMU or barometer. For a separate barometer:"));
body.push(table(
  ["Part", "Key specification (datasheet)", "Price", "Source"],
  [
    ["TE MS5611-01BA03", "Resolution 0.012 mbar RMS (1.2 Pa) at OSR 4096; 10 cm altitude resolution", "$12.32 (1 pc), $7.57 (100), $5.93 (1,000), $5.78 (reel 3,600)", "[15][16]"],
    ["Bosch BMP390", "Relative accuracy ±3 Pa (±25 cm); RMS noise 0.02 Pa at lowest bandwidth", "from $5.60 (chip)", "[17][18]"],
    ["Infineon DPS310", "Precision ±0.2 Pa (±2 cm) in high-precision mode; absolute ±100 Pa", "Adafruit breakout $6.95 (1–9), $5.56 (100+)", "[19]"],
  ],
  [1700, 3200, 2600, 1526]
));
body.push(caption("Table 3. Barometer candidates."));
body.push(B("All three are quieter than the simulator assumes: the MS5611 by a factor of about 8 (1.2 Pa against 10 Pa). Our simulated results are conservative on barometer noise."));
body.push(B("In flight, weather, temperature and rotor downwash limit the barometer more than the chip does; the simulator's drift term stands in for them."));
body.push(H2("2.2 Cameras"));
body.push(P("The final drone has two cameras, down and forward, each 1024×1024 px with a 90° field of view at 25 Hz. Our plan assumes the drone already carries a camera (docs/PLAN.md [46]). For a drone without them, two reference parts: the Raspberry Pi Global Shutter Camera (Sony IMX296, 1456×1088, $50 [36], lens extra and not priced here) and the Camera Module 3 wide ($35 [14]). Two cameras: about $70–100 plus lenses."));

// ===== 3 Sun sensor
body.push(H1("3. Sun sensor"));
body.push(H2("3.1 How accurate does it need to be?"));
body.push(P("The sun sensor measures the sun's direction in the drone's frame; with time and rough position the sun's direction in the world is known, and the difference gives the heading. On a drone two errors add up (baseline/src/sensors/heading.py [46]): the sensor error d, magnified by 1/cos(e) at sun elevation e, and the IMU's tilt error t, which shifts the sun sideways by t·tan(e). The heading error is √((d/cos e)² + (t·tan e)²). With the 1° tilt error our simulator assumes:"));
body.push(table(
  ["Sun elevation", "d = 0.03° (miniDSS)", "d = 0.1° (Fan et al., our model)", "d = 0.3° (uncalibrated miniDSS)", "d = 1.0°"],
  [["30°", "0.58°", "0.59°", "0.67°", "1.29°"], ["45°", "1.00°", "1.01°", "1.09°", "1.73°"], ["60°", "1.73°", "1.74°", "1.83°", "2.65°"]],
  [1500, 1850, 2050, 2050, 1576]
));
body.push(caption("Table 4. Heading error from a sun sensor on a drone with 1° tilt error, for four sensor accuracies (computed from the formula above)."));
body.push(P("**Consequence:** below about 0.3° the sensor no longer matters; the tilt error dominates. A sensor of 0.1–0.3° is sufficient. In our simulations the sun heading gave 0.9° median heading error on flight 1 (compass: 1.8° with fixed offsets up to 8° per draw), and in the fused filter it reduced the heading error from 55° to 1.3° (docs/simulation-results.md [46]). The five-photodiode variant was about as good as the compass and failed the 120 m flight, so it is not recommended. The final configuration keeps the digital sensor (heading.source: sun_digital)."));
body.push(H2("3.2 The paper: Micro-Digital Sun Sensor (Xie et al., 2010)"));
body.push(P("The Micro-Digital Sun Sensor (μDSS) was developed by TU Delft and TNO for micro-satellites [1][2]. Its core is the APS+, one CMOS chip that holds a 512×512 active pixel sensor (the test chip in the paper: 368×368), a 12-bit ADC, timing, and on-chip centroid computation, made in a standard 0.18 μm CMOS process. Sunlight passes through a membrane pinhole onto the array; the sun spot is about 10 pixels wide. To save power the chip first reads one row profile and one column profile (in effect two line sensors) to find the sun, then reads only a small window around it [2]."));
body.push(P("The concept was carried into the TNO miniDSS prototype [3]: 368×368 pixels of 6.5 μm on a 5×5 mm ASIC, 0.03° accuracy (0.3° without unit calibration), 0.01° noise-equivalent angle (3σ), 102°×102° field of view, output at 0.5–10 Hz, 72 g, 65 mW, designed for low recurrent cost."));
body.push(P("**Note on \"line sensor\":** the μDSS uses an area sensor (APS), not a line sensor; only its acquisition mode reads row and column profiles. The sensor in our simulator, Fan et al. (2016), does use a linear detector behind a V-shaped slit [4][5]. Both principles are costed below. **Price of the μDSS itself:** none; it is a research prototype, as is the miniDSS. The closest prices are those of commercial space sun sensors (Table 5)."));
body.push(table(
  ["Product", "Type", "Specification", "Price", "Source"],
  [
    ["Solar MEMS nanoSSOC-A60", "Analog, 2-axis", "3.7 g, 27.4 × 14 × 5.9 mm", "€2,500", "[6]"],
    ["Lens R&D BiSon64", "Analog, space-qualified", "catalogue 2018", "€4,317 (1–5 pcs); €3,358 in volume", "[7]"],
    ["Lens R&D BiSon64-ET FM", "Extended temperature, flight model", "catalogue 2018", "€8,984 (1–5 pcs); €7,007 (60+)", "[7]"],
    ["NewSpace NFSS-411", "Digital fine sun sensor", "0.1° RMS, 140° FOV, 5 Hz", "$12,000", "[8]"],
  ],
  [2200, 1700, 1900, 2000, 1226]
));
body.push(caption("Table 5. Space sun sensors. Their price pays for radiation tolerance, thermal-vacuum qualification and flight heritage, none of which a drone needs."));
body.push(H2("3.3 Building one from commercial parts"));
body.push(P("The mask, the neutral-density filter (sunlight saturates a bare sensor; the miniDSS needed an attenuating window [3]) and the housing have no public price; we **estimate** $2–10 for them together."));
body.push(table(
  ["Option", "Parts", "Sourced price", "Estimate (mask, filter, housing)", "Total per unit"],
  [
    ["A. Line sensor + V-slit (Fan et al., our simulated model)", "ams TSL1401CL 128×1 linear array; RP2040 microcontroller", "TSL1401CL $5.49–8.06 (1 pc), $2.50–3.71 (1,000) [9][10]; RP2040 $0.70–1 [12]", "$2–10", "**≈ $9–19** (1 pc); ≈ $5–15 in volume"],
    ["A+. Same, higher-grade line sensor", "Hamamatsu S11639-01 (2,048 px)", "€224.72–231.67 [11]", "$2–10", "≈ $250–265"],
    ["B. Pinhole + area sensor (μDSS principle)", "OV9281 1 MP global-shutter module; processing on the companion computer", "≈ £25 (≈ $32) [13]", "$2–10", "≈ $34–42"],
    ["C. Upward camera (Alessandro's sun detector)", "Raspberry Pi Camera Module 3", "$25–35 [14]", "$2–10 (filter)", "≈ $27–45"],
  ],
  [2000, 2100, 2200, 1300, 1426], { highlight: [0] }
));
body.push(caption("Table 6. Sun sensor built from commercial parts. Option A is the cheapest and is the type our simulator models."));
body.push(B("**Accuracy is not yet shown.** Fan et al. reached 0.1° and the miniDSS 0.03° in their laboratories [3][4]. A build from commercial parts has not been tested by us; the requirement from Table 4 (≤ 0.3°) leaves a margin of three to ten times."));
body.push(B("**Mass and power.** Fan et al.: 35 g, 200 mW [4]; miniDSS: 72 g in a space housing, 65 mW [3]. A drone build should be lighter; we have not weighed one."));
body.push(B("**Operational limits** (from our model): no heading when the sun is hidden by cloud (the gyro carries the heading), outside the field of view, or near the zenith (in Taiwan around midday from May to July). The magnetometer on the flight controller (IST8310 [20]) remains the fallback."));

// ===== 4 Range finder and LiDAR
body.push(H1("4. Downward range finder and LiDAR"));
body.push(H2("4.1 The final range finder"));
body.push(P("The final drone carries one narrow downward beam next to the down camera: 0.20–100 m, 1 cm resolution, σ 0.02 m, 25 Hz (model.sdf; launch options range_min_m, range_max_m, range_noise_std_m [46]). It scales the down camera's optical flow into a metric ground speed. The measurement is off by default (metric_flow:=false). Over flat ground (terrain world, 39 m) its speed readings were 0.16 m/s off in the median and the filter held 14.5 m median for 164 s, against 322 m without it; over the city (buildings 16–63 m, flight at 80 m) the speed was 4.45 m/s off because range and tracked points lay at different depths (docs/simulation-results.md [46])."));
body.push(P("**Note:** a 100 m limit does not reach the ground from our 110 m and 120 m Wufeng flights. If the range finder is to serve those heights, the limit must be raised and a longer-range part chosen (Table 7, last rows of the single-beam group)."));
body.push(H2("4.2 Candidates"));
body.push(table(
  ["Part", "Type", "Range", "Accuracy / mass", "Price", "Matches final spec?", "Source"],
  [
    ["Benewake TF02-Pro", "single beam", "40 m", "— / 50 g", "€94–120", "no (range)", "[22]"],
    ["LightWare LW20/C", "single beam", "0.2–100 m", "±5 cm / 19 g", "$279", "**yes, one-to-one**", "[23][24]"],
    ["Benewake TF03-180", "single beam", "100 m operating, 180 m max", "— / —", "$249.90–274.95", "yes (range)", "[25]"],
    ["LightWare SF30/C", "single beam", "100 m", "— / —", "$299", "yes (range)", "[23]"],
    ["LightWare SF30/D", "single beam", "200 m", "— / 36 g", "$399", "yes, also above 100 m", "[23]"],
    ["LightWare GRF-500", "single beam", "0.2–500 m", "— / 10.7 g, 0.6 W", "€524.95 incl. VAT (≈ €441 net)", "yes, also above 100 m", "[26][27]"],
    ["Livox Mid-360", "scanning", "40 m at 10 % reflectivity", "— / 265 g", "≈ $1,092", "no (range)", "[28][29]"],
    ["Livox Avia", "scanning, triple return", "up to 450 m", "— / 498 g", "$2,079 (US); AUD 4,290", "different sensor class", "[30][31]"],
    ["Ouster OS1-32", "scanning", "—", "—", "$5,770 (eBay) – $8,000 (2019 list)", "different sensor class", "[32]"],
    ["Jenoptik DLEM 20", "laser range finder module", "up to 5 km", "±0.5 m / < 33 g", "not published", "TRN class", "[33]"],
  ],
  [1450, 1100, 1450, 1250, 1600, 1350, 826], { highlight: [1] }
));
body.push(caption("Table 7. Range finder and LiDAR candidates. \"—\" = not taken from a source in this report."));
body.push(B("**Recommended:** LightWare LW20/C, $279: its range is exactly the simulator's 0.2–100 m, ±5 cm is comparable to the simulated σ 2 cm, and it reads 48 to 5,000 times a second, well above the 25 Hz used [24]. Cheaper with the same reach: TF03-180 at $250–275 [25]. For flights above 100 m: SF30/D ($399) or GRF-500 (≈ €441 net)."));
body.push(B("**Scanning LiDAR** (LiDAR-inertial odometry) is not part of our design. The affordable Mid-360 does not see the ground from 65–120 m; the Avia does, at ≈ $2,500 including a Jetson Orin Nano Super ($399 [34]) and about 0.5 kg."));
body.push(B("**Felix's laser TRN** (σ 0.3 m, 10 Hz, up to 5 km, three echoes; TRN/configs/laser.yaml) needs a km-class module such as the DLEM 20; its price must be requested."));
body.push(B("**Every LiDAR emits:** 905 nm pulses are visible to night-vision and near-infrared cameras and to laser warning receivers. The cameras and the sun sensor are passive; the AIS receiver only listens."));

// ===== 5 RF direction finder
body.push(H1("5. RF direction finder (AIS, over water)"));
body.push(H2("5.1 The final design"));
body.push(P("Over open water the camera has nothing to match. The final drone then takes bearings to ships' AIS transmitters (161.975 and 162.025 MHz) with a pseudo-Doppler direction finder: four 17 cm helical VHF stubs on the rotor arms (square of 13 cm side, 0.07 wavelength), switched one after another into a single receiver 1,000 times a second; the phase of the resulting Doppler tone gives the bearing. Sensitivity −110 dBm, noise figure 6 dB, bearing σ 3° plus a 1° mounting bias (sim/config/rf.yaml, sim/RF_README.md [46]). The design follows Gerhard and Tokekar (2020), who flew four antennas, an Opera Cake switch board and a HackRF One on a four-rotor UAS at 150 MHz [37]. In the latest strait-world run the ships' bearings alone gave 55 m median error over 3.5 minutes (docs/simulation-results.md [46])."));
body.push(H2("5.2 Cost"));
body.push(table(
  ["Item", "Reference build (as in [37])", "Low-cost build", "Source"],
  [
    ["Receiver", "HackRF One, €305", "RTL-SDR Blog V4, $39.95 (with antennas, launch price) – $84.95 (SparkFun kit)", "[39][40][41]"],
    ["RF switch", "Opera Cake, €189 / US$190", "Analog Devices ADG904 SP4T, $5.88 (1 pc), $3.30 in volume", "[38][39][43]"],
    ["Switch timing", "Opera Cake firmware", "RP2040 microcontroller, $0.70–1", "[12]"],
    ["4 antennas", "4 × Nagoya NA-701 (22 cm VHF/UHF whip), €15.95 each", "4 × Nagoya NA-701, $14.99–24.99 each", "[44]"],
    ["PCB, coax, mounts", "—", "**estimate** $5–15", "—"],
    ["**Total**", "**≈ €558 (≈ $615)**", "**≈ $110–210**", ""],
  ],
  [1600, 2900, 3300, 1226], { highlight: [5] }
));
body.push(caption("Table 8. Cost of the AIS direction finder, as a copy of the published hardware and as a low-cost build."));
body.push(B("**Antenna caveat.** The simulator assumes 17 cm helical stubs. The NA-701 is a 22 cm dual-band whip; marine AIS helical stubs of 15 cm exist but are sold with brackets and 20 m of cable at $85.49 or more each [45], which would add about $240–280 for four."));
body.push(B("**Receiver caveat.** The low-cost build relies on the switching being synchronised with the RTL-SDR's samples; the published system used the HackRF and the Opera Cake [37]. It has not been built or tested by us."));
body.push(B("**Alternative:** a coherent five-channel receiver (KrakenSDR, $499 [42]) measures bearings by interferometry instead of switching. It costs more and is larger than the switched design."));

// ===== 6 Configurations
body.push(H1("6. Cost per drone: configurations compared"));
body.push(table(
  ["Configuration", "Added hardware", "Added cost (≈ USD)", "Share of a $500 drone"],
  [
    ["1. Core navigator (cameras, IMU, barometer on the drone)", "Sun sensor, option A", "$10–20", "2–4 %"],
    ["2. Final full suite, low-cost radio", "Sun sensor + LW20/C (or TF03) + low-cost AIS direction finder", "$370–530", "74–106 %"],
    ["2b. Final full suite, reference radio", "as 2, radio as in Gerhard and Tokekar", "$875–935", "175–187 %"],
    ["3. Platform parts, if the drone has none", "Pixhawk 6C + 2 cameras + Raspberry Pi 5 (4 GB)", "$367–397 (plus lenses)", "73–79 %"],
    ["4. Scanning LiDAR instead of the range finder", "Livox Avia + Jetson Orin Nano Super", "≈ $2,480", "≈ 500 %"],
    ["5. High-end scanning LiDAR", "Ouster OS1-32 + Jetson", "$6,170–8,400", "1,230–1,680 %"],
    ["6. Space-grade sun sensor instead of option A", "nanoSSOC-A60 … NFSS-411", "$2,750–12,000", "550–2,400 %; no accuracy benefit (Table 4)"],
  ],
  [3100, 2700, 1500, 1726], { highlight: [0, 1] }
));
body.push(caption("Table 9. Added hardware cost per drone, from Tables 1–8. Conversions at the rates in section 1.2; the $500 airframe is the orientation agreed with our mentor (docs/PLAN.md [46])."));

// ===== 7 Conclusions
body.push(H1("7. Conclusions and recommendations"));
body.push(N("**Core navigator:** about $10–20 of new hardware, the sun sensor. Cameras, IMU and barometer of the final version are standard on a Pixhawk-class drone."));
body.push(N("**Sun sensor:** build option A (TSL1401CL behind a V-slit, read by an RP2040) and bench-test it against a known sun direction; the requirement is ≤ 0.3°. Do not buy a space-grade sensor."));
body.push(N("**Range finder:** if metric flow is used, the LightWare LW20/C ($279) matches the final specification; for flights above 100 m use the SF30/D or GRF-500."));
body.push(N("**AIS direction finder:** a low-cost build is about $110–210 but untested; the published reference hardware costs about $615. Build and bench-test one before quoting a number to customers."));
body.push(N("**For the pitch:** the core adds about $10–20 per drone; the complete final suite about $370–530; a scanning LiDAR alone about $2,500. The sensors are passive except the optional range finder."));

// ===== 8 Limitations
body.push(H1("8. Limitations"));
body.push(B("List prices of one day; distributor prices, stock and exchange rates change. Defence volumes and export-controlled items are priced on request."));
body.push(B("Mechanics, PCBs, cables and assembly of the self-built sun sensor and direction finder are estimates, not sourced prices."));
body.push(B("The accuracy of the self-built sun sensor and of the low-cost direction finder is assumed from the literature, not measured by us."));
body.push(B("Specifications are as published by manufacturers or retailers; we did not test the parts. Some secondary listings (eBay, regional retailers) were used where no primary price was available; these are marked by source."));
body.push(B("The specifications come from the simulator; they are what our results assume, not a flight-qualified design."));

body.push(H1("References"));
body.push(...refParas);

const doc = new Document({
  creator: "Team Taipei Drift", title: "Sensor Cost Report", description: "Barometer, sun sensor and LiDAR costs for GNSS-free drone navigation",
  styles: {
    default: { document: { run: { font: FONT, size: 21 } } },
    paragraphStyles: [
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true, run: { size: 30, bold: true, font: FONT, color: "1F2D3D" }, paragraph: { spacing: { before: 360, after: 160 }, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true, run: { size: 24, bold: true, font: FONT, color: "3A424D" }, paragraph: { spacing: { before: 240, after: 120 }, outlineLevel: 1 } },
    ],
  },
  numbering: {
    config: [
      { reference: "bullets", levels: [{ level: 0, format: LevelFormat.BULLET, text: "•", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 567, hanging: 283 } } } }] },
      { reference: "numbers", levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 567, hanging: 283 } } } }] },
    ],
  },
  sections: [{
    properties: { page: { size: { width: 11906, height: 16838 }, margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.RIGHT, children: [new TextRun({ text: "Taipei Drift | Sensor Cost Report | page ", size: 16, color: "777777" }), new TextRun({ children: [PageNumber.CURRENT], size: 16, color: "777777" })] })] }) },
    children: body,
  }],
});

Packer.toBuffer(doc).then((buf) => { fs.writeFileSync(OUT, buf); console.log("wrote", OUT); });

const net = require("net");

const PORT = Number(process.env.PORT ?? 8899);
const HOST = String(process.env.HOST ?? "0.0.0.0");

// ===== PV emulation (MODE affects ONLY these) =====
// 0x1023 Photovoltaic charging power (W)
const DAY_PV_POWER_W = Number(process.env.DAY_ACTIVE_POWER_W ?? 12000);
const NIGHT_PV_POWER_W = Number(process.env.NIGHT_ACTIVE_POWER_W ?? 800);

// 0x1020 PV panel voltage (0.1V)
const DAY_PV_VOLT_01V = Number(process.env.DAY_PV_VOLTAGE_01V ?? 900); // 90.0V
const NIGHT_PV_VOLT_01V = Number(process.env.NIGHT_PV_VOLTAGE_01V ?? 0); // 0.0V

// ===== Customer setting data (GLOBAL, not per MODE) =====
// your battery-string defaults (only for initial default)
const BATTERY_STRING = Number(process.env.BATTERY_STRING ?? 2);
const V_DAY_ON = 131 * BATTERY_STRING;  // default only
const V_DAY_OFF = 115 * BATTERY_STRING; // default only
var BAT_VOL = 274; //battery voltage
const AC_LOST = 0; //ac voltage
const AC_NORM = 2300;

let ON_SET = Number(process.env.ON_SET ?? V_DAY_ON);   // reg 0x1037
let OFF_SET = Number(process.env.OFF_SET ?? V_DAY_OFF); // reg 0x1038

// ===== Manual mode  =====
let MODE = String(process.env.MODE ?? "day").toLowerCase();
if (MODE !== "day" && MODE !== "night") MODE = "day";

let AC = String(process.env.AC ?? "acok").toLowerCase();
if (AC !== "aclost" && AC !== "acok") AC = "acok";

function curPvPowerW() {
  return MODE === "day" ? DAY_PV_POWER_W : NIGHT_PV_POWER_W;
}
function curPvVolt01V() {
  return MODE === "day" ? DAY_PV_VOLT_01V : NIGHT_PV_VOLT_01V;
}
function curBatVol() {
    return BAT_VOL;
}
function curAC() {
  return AC === "acok" ? AC_NORM : AC_LOST;
}

// ---- CRC16 Modbus (poly 0xA001), returns uint16 ----
function crc16modbus(buf) {
  let crc = 0xffff;
  for (const b of buf) {
    crc ^= b;
    for (let i = 0; i < 8; i++) {
      const lsb = crc & 1;
      crc >>= 1;
      if (lsb) crc ^= 0xa001;
    }
  }
  return crc & 0xffff;
}
function withCrc(payloadNoCrc) {
  const crc = crc16modbus(payloadNoCrc);
  const out = Buffer.alloc(payloadNoCrc.length + 2);
  payloadNoCrc.copy(out, 0);
  out[out.length - 2] = crc & 0xff; // LSB
  out[out.length - 1] = (crc >> 8) & 0xff; // MSB
  return out;
}

function hexToBuf(hex) {
  return Buffer.from(hex.replace(/[^0-9a-f]/gi, ""), "hex");
}
function u16be(buf, off) {
  return (buf[off] << 8) | buf[off + 1];
}
function fmtU16(v) {
  return "0x" + (v >>> 0).toString(16).padStart(4, "0");
}

// ---- request match helpers ----
function isReq(buf, addr, func, startHi, startLo, qtyHi, qtyLo) {
  return (
    buf.length >= 8 &&
    buf[0] === addr &&
    buf[1] === func &&
    buf[2] === startHi &&
    buf[3] === startLo &&
    buf[4] === qtyHi &&
    buf[5] === qtyLo
  );
}

// Modbus exception response: addr, (func|0x80), code, crc
function exception(addr, func, code) {
  return withCrc(Buffer.from([addr, func | 0x80, code]));
}

// ===== Standard responses you provided (with CRC at end) =====
const RUN_STD_HEX =
  "06 12 4C 11 00 03 C7 00 00 08 FF 08 FF 00 00 00 02 00 06 01 12 00 E7 00 00 00 00 13 78 00 00 00 04 9F F7 00 00 3A E7 00 00 00 02 00 01 00 00 00 A1 00 00 00 01 0C 1C 08 FC 00 00 00 00 00 00 00 00 00 00 00 00 00 01 03 6A 00 02 00 03 00 37 00 66";

const BASIC_STD_HEX =
  "06 16 52 11 00 03 C7 01 36 01 2A 01 2C 01 20 01 08 01 14 00 CC 01 06 00 D8 00 01 00 03 00 02 00 01 00 06 9F F7 06 F4 07 3A 09 C4 0A 3E 01 06 00 E6 00 01 00 00 00 02 01 F4 00 C8 00 01 17 70 00 FA 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 78 D6";

const RUN_STD = hexToBuf(RUN_STD_HEX);
const BASIC_STD = hexToBuf(BASIC_STD_HEX);

// templates without CRC (we always recalc CRC)
const RUN_TPL = Buffer.from(RUN_STD.subarray(0, RUN_STD.length - 2));
const BASIC_TPL = Buffer.from(BASIC_STD.subarray(0, BASIC_STD.length - 2));

// Patch helper: patch 16-bit BE register inside response frame
// Frame layout: [addr func bytecount][deviceNo(4 bytes)][regs...]
function patchU16RegInResp(frameNoCrc, baseReg, regAddr, valueU16) {
  const regsStart = 3 + 4;
  const byteCount = frameNoCrc[2];
  const regsCount = byteCount / 2;

  const idx = regAddr - baseReg;
  if (idx < 0 || idx >= regsCount) return false;

  const off = regsStart + idx * 2;
  const v = Math.max(0, Math.min(0xffff, Number(valueU16) | 0));
  frameNoCrc[off] = (v >> 8) & 0xff;
  frameNoCrc[off + 1] = v & 0xff;
  return true;
}

// ===== Build responses (dynamic) =====
function buildRunResp() {
  const frame = Buffer.from(RUN_TPL);

  // RUN block base = 0x1000 (your earlier spec)
  patchU16RegInResp(frame, 0x1000, 0x1020, curPvVolt01V()); // PV voltage 0.1V
  patchU16RegInResp(frame, 0x1000, 0x1023, curPvPowerW());  // PV power W
  patchU16RegInResp(frame, 0x1000, 0x1006, curBatVol());  // Battery voltage
  patchU16RegInResp(frame, 0x1000, 0x1001, curAC());  // AC mains voltage

  return withCrc(frame);
}



function buildBasicResp() {
  // BASIC block base = 0x1024 (your request start)
  const frame = Buffer.from(BASIC_TPL);

  patchU16RegInResp(frame, 0x1024, 0x1037, ON_SET);
  patchU16RegInResp(frame, 0x1024, 0x1038, OFF_SET);

  return withCrc(frame);
}

// ===== write handler (func 0x18, device-specific "write single reg") =====
function handleWrite18(req8) {
  const reg = u16be(req8, 2);
  const val = u16be(req8, 4);

  if (reg !== 0x1037 && reg !== 0x1038) {
    console.log("-> WRITE: unsupported reg", fmtU16(reg), "val", val);
    return exception(0x06, 0x18, 0x02); // illegal data address
  }

  if (reg === 0x1037) ON_SET = val;
  else OFF_SET = val;

  // ACK = echo first 6 bytes + CRC
  const body = req8.subarray(0, 6);
  const resp = withCrc(body);

  console.log(`-> WRITE OK ${fmtU16(reg)}=${val} (global)`);
  return resp;
}

// ===== CLI =====
function printStatus() {
  console.log(
    `[MODE=${MODE}] PVvolt(0x1020)=${curPvVolt01V() / 10}V PVpower(0x1023)=${curPvPowerW()}W | ` +
    `Customer(0x1037/0x1038) ON/OFF=${ON_SET}/${OFF_SET}`
  );
}

process.stdin.setEncoding("utf8");
process.stdin.on("data", (d) => {
  const line = String(d).trim();
  if (!line) return;

  const [cmdRaw, argRaw] = line.split(/\s+/, 2);
  const cmd = String(cmdRaw).toLowerCase();

  if (cmd === "day" || cmd === "night") {
    MODE = cmd;
    console.log("OK -> mode set:", MODE);
    printStatus();
    return;
  }
  if (cmd === "status" || cmd === "s") {
    printStatus();
    return;
  }
  if (cmd === "seton" || cmd === "setoff") {
    const v = Number(argRaw);
    if (!Number.isFinite(v) || v < 0 || v > 65535) {
      console.log("Bad value. Example: seton 262");
      return;
    }
    if (cmd === "seton") ON_SET = v;
    else OFF_SET = v;

    console.log("OK ->", cmd, v, "(global customer settings)");
    printStatus();
    return;
  }
  if (cmd === "acok" || cmd === "aclost") {
    AC = cmd;
    console.log("OK ->", cmd, "(global customer settings)");
    printStatus();
    return;
  }
  if (cmd === "bat") {
    const v = Number(argRaw);
    if (!Number.isFinite(v) || v < 0 || v > 65535) {
      console.log("Bad value. Example: bat 262");
      return;
    }
    BAT_VOL = v;

    console.log("OK ->", cmd, v, "(global customer settings)");
    printStatus();
    return;
  }
  if (cmd === "help" || cmd === "h" || cmd === "?") {
    console.log("Commands: day | night | status | seton <u16> | setoff <u16> | help | bat <volatge> | aclost | acok");
    return;
  }
  console.log("Unknown command. Use: help");
});

// ===== TCP server =====
const server = net.createServer((sock) => {
  console.log("client connected:", sock.remoteAddress, sock.remotePort);
  sock.setNoDelay(true);

  let acc = Buffer.alloc(0);

  sock.on("data", (chunk) => {
    acc = Buffer.concat([acc, chunk]);

    // requests are 8 bytes
    while (acc.length >= 8) {
      const req = acc.subarray(0, 8);
      acc = acc.subarray(8);

      const body = req.subarray(0, 6);
      const crcGot = req[6] | (req[7] << 8);
      const crcCalc = crc16modbus(body);

      console.log("REQ:", req.toString("hex").match(/../g).join(" "));
      if (crcGot !== crcCalc) {
        console.log("bad CRC, got", crcGot.toString(16), "calc", crcCalc.toString(16));
        continue;
      }

      // readRunParams: 06 12 10 00 00 24 ...
      if (isReq(req, 0x06, 0x12, 0x10, 0x00, 0x00, 0x24)) {
        const runResp = buildRunResp();
        console.log("-> RUN_RESP mode=", MODE, "pv01v=", curPvVolt01V(), "pvpwr=", curPvPowerW());
        sock.write(runResp);
        continue;
      }

      // readBasicParams: 06 16 10 24 00 27 ...
      if (isReq(req, 0x06, 0x16, 0x10, 0x24, 0x00, 0x27)) {
        const basicResp = buildBasicResp();
        console.log("-> BASIC_RESP ON/OFF=", ON_SET, OFF_SET);
        sock.write(basicResp);
        continue;
      }

      // write single register (device uses func=0x18)
      if (req[0] === 0x06 && req[1] === 0x18) {
        const resp = handleWrite18(req);
        sock.write(resp);
        continue;
      }

      console.log("unknown request");
    }
  });

  sock.on("close", () => console.log("client closed"));
  sock.on("error", (e) => console.log("socket error", e.message));
});

server.listen(PORT, HOST, () => {
  console.log(`JNGE mock server listening on ${HOST}:${PORT}`);
  printStatus();
  console.log("Type: day | night | status | seton <u16> | setoff <u16> | help");
});
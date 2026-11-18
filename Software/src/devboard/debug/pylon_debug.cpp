#include "pylon_debug.h"
#include "../../battery/BATTERIES.h"
#include "../../battery/PYLON-DEBUG.h"
#include "../../datalayer/datalayer.h"
#include "../safety/safety.h"

/* Bench console for the Pylon relay control. Nothing here is stored to flash: a reboot returns to
   the saved settings. 0x8240 (mask external comm loss) and 0x8260 (Run) are refused, as in the
   driver: they would bypass the BMS protections. */

static const char* const HELP =
    "Pylon debug console. Commands:\n"
    "  status                     driver state, as on the More battery info page\n"
    "  tx <id> <hex bytes>        send once, e.g. tx 8201 55  (29-bit ID, padded to 8 bytes)\n"
    "  per <slot 0-7> <ms> <id> <hex bytes>   repeat every <ms>\n"
    "  per <slot> off | per off   stop one / all repeats\n"
    "  mute none|relay|all        relay: driver skips its 0x820x/0x821x/0x822x frames\n"
    "                             all: driver sends nothing (no polling either)\n"
    "  mode 0|1|2                 relay control mode until reboot (0 Off, 1 Force, 2 Sleep/Wake)\n"
    "  addr 0-15                  BMS address until reboot (0 = auto)\n"
    "  delay <ms>                 relay close delay until reboot\n"
    "  open | close               same as the Open/Close Contactors button, not stored\n";

static String next_token(const String& s, int& pos) {
  while (pos < (int)s.length() && s[pos] == ' ') {
    pos++;
  }
  int start = pos;
  while (pos < (int)s.length() && s[pos] != ' ') {
    pos++;
  }
  return s.substring(start, pos);
}

static bool parse_hex_u32(const String& s, uint32_t& out) {
  String t = s;
  if (t.startsWith("0x") || t.startsWith("0X")) {
    t = t.substring(2);
  }
  if (t.length() == 0 || t.length() > 8) {
    return false;
  }
  out = 0;
  for (size_t i = 0; i < t.length(); i++) {
    char c = t[i];
    uint8_t v;
    if (c >= '0' && c <= '9') {
      v = c - '0';
    } else if (c >= 'a' && c <= 'f') {
      v = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
      v = c - 'A' + 10;
    } else {
      return false;
    }
    out = (out << 4) | v;
  }
  return true;
}

// <id> <hex bytes...> starting at pos; the bytes may be split by spaces ("55 00" or "5500")
static bool parse_frame(const String& line, int pos, CAN_frame& frame, String& error) {
  uint32_t id;
  if (!parse_hex_u32(next_token(line, pos), id) || id > 0x1FFFFFFF) {
    error = "bad CAN ID";
    return false;
  }
  if ((id & 0xFFFFFFF0) == 0x8240 || (id & 0xFFFFFFF0) == 0x8260) {
    error = "0x824x and 0x826x are refused (they bypass the BMS protections)";
    return false;
  }
  String hex;
  for (int i = pos; i < (int)line.length(); i++) {
    if (line[i] != ' ') {
      hex += line[i];
    }
  }
  if (hex.length() % 2 != 0 || hex.length() > 16) {
    error = "data must be 0-8 whole bytes in hex";
    return false;
  }
  frame = {};
  frame.FD = false;
  frame.ext_ID = true;
  frame.DLC = 8;
  frame.ID = id;
  for (size_t i = 0; i < hex.length() / 2; i++) {
    uint32_t b;
    if (!parse_hex_u32(hex.substring(i * 2, i * 2 + 2), b)) {
      error = "bad data byte";
      return false;
    }
    frame.data.u8[i] = b;
  }
  return true;
}

static String frame_text(const CAN_frame& f) {
  char buf[48];
  snprintf(buf, sizeof(buf), "%08lX [%u]", (unsigned long)f.ID, f.DLC);
  String s = buf;
  for (uint8_t i = 0; i < f.DLC && i < 8; i++) {
    snprintf(buf, sizeof(buf), " %02X", f.data.u8[i]);
    s += buf;
  }
  return s;
}

static String html_to_text(String html) {
  html.replace("</h4>", "\n");
  html.replace("<br>", "\n");
  html.replace("&nbsp;", " ");
  String out;
  bool in_tag = false;
  for (size_t i = 0; i < html.length(); i++) {
    char c = html[i];
    if (c == '<') {
      in_tag = true;
    } else if (c == '>') {
      in_tag = false;
    } else if (!in_tag) {
      out += c;
    }
  }
  return out;
}

static String status_text() {
  String s;
  s += "uptime_ms " + String(millis()) + "\n";
  s += "voltage_dV " + String(datalayer.battery.status.voltage_dV) + "  current_dA " +
       String(datalayer.battery.status.current_dA) + "  soc_pptt " + String(datalayer.battery.status.real_soc) + "\n";
  s += "inverter_allows_closing " + String(datalayer.system.status.inverter_allows_contactor_closing) +
       "  equipment_stop " + String(datalayer.system.info.equipment_stop_active) + "  dc_bus_live " +
       String(datalayer.system.status.dc_bus_live) + "  can_alive " +
       String(datalayer.battery.status.CAN_battery_still_alive) + "\n";
  s += "mode " + String(user_selected_pylon_relay_control) + "  addr " + String(user_selected_pylon_bms_address) +
       "  delay_ms " + String(user_selected_pylon_relay_close_delay_ms) + "  mute " + String(pylon_debug.mute) + "\n";
  for (int i = 0; i < PylonDebug::PERIODIC_SLOTS; i++) {
    PylonDebug::Periodic p = pylon_debug.get_periodic(i);
    if (p.active) {
      s += "per " + String(i) + " every " + String(p.period_ms) + " ms: " + frame_text(p.frame) + "\n";
    }
  }
  if (battery) {
    s += html_to_text(battery->get_status_renderer().get_status_html());
  }
  return s;
}

String pylon_debug_command(const String& raw_line) {
  String line = raw_line;
  line.trim();
  line.replace("\t", " ");
  int pos = 0;
  String cmd = next_token(line, pos);
  cmd.toLowerCase();

  if (cmd == "" || cmd == "help" || cmd == "?") {
    return HELP;
  }
  if (cmd == "status") {
    return status_text();
  }
  if (cmd == "tx") {
    CAN_frame frame;
    String error;
    if (!parse_frame(line, pos, frame, error)) {
      return "ERR " + error + "\n";
    }
    if (!pylon_debug.enqueue(frame)) {
      return "ERR queue full\n";
    }
    return "OK tx " + frame_text(frame) + "\n";
  }
  if (cmd == "per") {
    String slot_text = next_token(line, pos);
    if (slot_text == "off") {
      pylon_debug.clear_periodic();
      return "OK all repeats off\n";
    }
    int slot = slot_text.toInt();
    if (slot_text.length() == 0 || slot < 0 || slot >= PylonDebug::PERIODIC_SLOTS) {
      return "ERR slot must be 0-7\n";
    }
    String period_text = next_token(line, pos);
    if (period_text == "off") {
      pylon_debug.set_periodic(slot, 0, nullptr);
      return "OK repeat " + String(slot) + " off\n";
    }
    long period = period_text.toInt();
    if (period < 10 || period > 60000) {
      return "ERR period must be 10-60000 ms\n";
    }
    CAN_frame frame;
    String error;
    if (!parse_frame(line, pos, frame, error)) {
      return "ERR " + error + "\n";
    }
    pylon_debug.set_periodic(slot, period, &frame);
    return "OK repeat " + String(slot) + " every " + String(period) + " ms: " + frame_text(frame) + "\n";
  }
  if (cmd == "mute") {
    String arg = next_token(line, pos);
    if (arg == "none" || arg == "0") {
      pylon_debug.mute = PylonDebug::MUTE_NONE;
    } else if (arg == "relay" || arg == "1") {
      pylon_debug.mute = PylonDebug::MUTE_RELAY;
    } else if (arg == "all" || arg == "2") {
      pylon_debug.mute = PylonDebug::MUTE_ALL;
    } else {
      return "ERR mute none|relay|all\n";
    }
    return "OK mute " + String(pylon_debug.mute) + "\n";
  }
  if (cmd == "mode") {
    String arg = next_token(line, pos);
    int mode = arg.toInt();
    if (arg.length() != 1 || mode < 0 || mode > 2) {
      return "ERR mode 0|1|2\n";
    }
    user_selected_pylon_relay_control = mode;
    return "OK mode " + String(mode) + " (until reboot)\n";
  }
  if (cmd == "addr") {
    String arg = next_token(line, pos);
    int addr = arg.toInt();
    if (arg.length() == 0 || addr < 0 || addr > 15) {
      return "ERR addr 0-15\n";
    }
    user_selected_pylon_bms_address = addr;
    return "OK addr " + String(addr) + " (until reboot)\n";
  }
  if (cmd == "delay") {
    String arg = next_token(line, pos);
    long ms = arg.toInt();
    if (arg.length() == 0 || ms < 0 || ms > 30000) {
      return "ERR delay 0-30000\n";
    }
    user_selected_pylon_relay_close_delay_ms = ms;
    return "OK delay " + String(ms) + " ms (until reboot)\n";
  }
  if (cmd == "open") {
    setBatteryPause(true, false, EquipmentStop::STOP, false);
    return "OK open requested (equipment stop on, not stored)\n";
  }
  if (cmd == "close") {
    setBatteryPause(false, false, EquipmentStop::RESUME, false);
    return "OK close requested (equipment stop off, not stored)\n";
  }
  return "ERR unknown command, try help\n";
}

void pylon_debug_poll_serial() {
  static String buffer;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c != '\n') {
      if (buffer.length() < 200) {
        buffer += c;
      }
      continue;
    }
    String reply = pylon_debug_command(buffer);
    Serial.print("DBG> " + buffer + "\n");
    reply.replace("\n", "\nDBG ");
    Serial.print("DBG " + reply + "\n");
    buffer = "";
  }
}

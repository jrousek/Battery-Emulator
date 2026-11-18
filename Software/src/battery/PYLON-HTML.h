#ifndef _PYLON_HTML_H
#define _PYLON_HTML_H

#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "../devboard/webserver/BatteryHtmlRenderer.h"

/** Extra data parsed from the Pylon HV protocol that has no datalayer home.
 *  Filled by PylonBattery, displayed by PylonHtmlRenderer. */
struct PylonExtendedData {
  uint16_t charge_cutoff_dV = 0;
  uint16_t discharge_cutoff_dV = 0;
  uint16_t cell_max_number = 0;
  uint16_t cell_min_number = 0;
  uint16_t temp_max_sensor = 0;
  uint16_t temp_min_sensor = 0;

  // BMS address and relay control (PYLONADDR / PYLONRELAY)
  bool address_is_auto = true;
  uint8_t bms_address_detected = 0;  // 0 = not seen yet
  uint8_t command_address = 1;       // Address the 0x82x0 commands go to (modes 1 and 2)
  uint8_t relay_control_mode = 0;
  bool want_closed = false;
  bool have_8230_response = false;
  unsigned long last_8230_response_millis = 0;
  uint8_t last_8230_data[8] = {0};

  // 0x4250+A and 0x4280+A
  bool have_4250 = false;
  uint8_t basic_status = 0;  // Byte0 bits 0-2: 0 sleep, 1 charge, 2 discharge, 3 idle
  uint8_t fault = 0;
  uint16_t alarm = 0;
  uint16_t protection = 0;
  uint8_t charge_forbidden = 0;
  uint8_t discharge_forbidden = 0;

  bool old_frames_seen = false;  // 0x4210+A
  bool new_frames_seen = false;  // 0x42B0+A (V1.27+)
};

class PylonHtmlRenderer : public BatteryHtmlRenderer {
 public:
  PylonHtmlRenderer(PylonExtendedData* d) : data(d) {}

  String get_status_html() {
    String content;
    content += "<h4>BMS charge cutoff voltage: " + String(data->charge_cutoff_dV / 10.0, 1) + " V</h4>";
    content += "<h4>BMS discharge cutoff voltage: " + String(data->discharge_cutoff_dV / 10.0, 1) + " V</h4>";
    content += "<h4>Design voltage in use: " + String(datalayer.battery.info.min_design_voltage_dV / 10.0, 1) + " - " +
               String(datalayer.battery.info.max_design_voltage_dV / 10.0, 1) + " V</h4>";
    if (data->cell_max_number > 0 || data->cell_min_number > 0) {
      content += "<h4>Highest cell: #" + String(data->cell_max_number) + " &nbsp; Lowest cell: #" +
                 String(data->cell_min_number) + "</h4>";
    }
    if (data->temp_max_sensor > 0 || data->temp_min_sensor > 0) {
      content += "<h4>Hottest sensor: #" + String(data->temp_max_sensor) + " &nbsp; Coldest sensor: #" +
                 String(data->temp_min_sensor) + "</h4>";
    }

    static const char* const relay_modes[] = {"Off (original)", "Force relay (0x822x)", "Sleep/Wake (0x820x)"};
    static const char* const basic_states[] = {"Sleep", "Charge", "Discharge", "Idle"};

    content += "<h4>BMS address: ";
    if (data->address_is_auto) {
      content += "auto, ";
      content += data->bms_address_detected ? "detected " + String(data->bms_address_detected) : String("not detected");
    } else {
      content += "manual";
    }
    content += ", commands to " + String(data->command_address) + "</h4>";
    content += "<h4>Frames received: ";
    content += data->old_frames_seen ? "old 0x421x " : "";
    content += data->new_frames_seen ? "new 0x42Bx (preferred)" : "";
    content += (!data->old_frames_seen && !data->new_frames_seen) ? "none" : "";
    content += "</h4>";
    content +=
        "<h4>Relay control: " + String(relay_modes[data->relay_control_mode <= 2 ? data->relay_control_mode : 0]) +
        "</h4>";
    content += "<h4>Relay requested: " + String(data->want_closed ? "closed" : "open") +
               " &nbsp; DC bus live: " + String(datalayer.system.status.dc_bus_live ? "yes" : "no") + "</h4>";
    content += "<h4>Last 0x8230 response: ";
    if (data->have_8230_response) {
      content += String((millis() - data->last_8230_response_millis) / 1000) + " s ago, data";
      for (uint8_t i = 0; i < 8; i++) {
        content += " " + hex2(data->last_8230_data[i]);
      }
    } else {
      content += "none";
    }
    content += "</h4>";
    if (data->have_4250) {
      content += "<h4>BMS basic status: " + String(data->basic_status <= 3 ? basic_states[data->basic_status] : "?") +
                 " (" + String(data->basic_status) + ")</h4>";
      content += "<h4>Fault: 0x" + hex2(data->fault) + (data->fault & 0x20 ? " (RELAY_ERR)" : "") +
                 " &nbsp; Alarm: 0x" + hex2(data->alarm >> 8) + hex2(data->alarm & 0xFF) + " &nbsp; Protection: 0x" +
                 hex2(data->protection >> 8) + hex2(data->protection & 0xFF) + "</h4>";
    } else {
      content += "<h4>BMS basic status: no 0x4250 received</h4>";
    }
    content += "<h4>Charge forbidden: " + String(data->charge_forbidden == 0xAA ? "yes" : "no") +
               " &nbsp; Discharge forbidden: " + String(data->discharge_forbidden == 0xAA ? "yes" : "no") + "</h4>";
    return content;
  }

 private:
  PylonExtendedData* data;

  static String hex2(uint8_t value) {
    static const char digits[] = "0123456789ABCDEF";
    String s;
    s += digits[value >> 4];
    s += digits[value & 0x0F];
    return s;
  }
};

#endif

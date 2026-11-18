#include "PYLON-BATTERY.h"
#include <string.h>
#include "../battery/BATTERIES.h"
#include "../communication/can/comm_can.h"
#include "../communication/contactorcontrol/comm_contactorcontrol.h"
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"  //For "More battery info" webpage
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"

/*Based on CAN-Bus-Protocol-Pylon-high-voltage-V1.26-20210903.pdf , which is the Pylontech 1.26 std.
  Address handling, 0x42B0-0x42D0, 0x8220/0x8230 and relay control from V1.29-20220926. */

PylonDebug pylon_debug;

void PylonBattery::update_values() {

  datalayer_battery->status.real_soc = (SOC * 100);  //increase SOC range from 0-100 -> 100.00

  datalayer_battery->status.soh_pptt = (SOH * 100);  //Increase decimals from 100% -> 100.00%

  datalayer_battery->status.voltage_dV = voltage_dV;  //value is *10 (3700 = 370.0)

  datalayer_battery->status.current_dA = current_dA;  //value is *10 (150 = 15.0) , invert the sign

  datalayer_battery->status.max_charge_power_W = current_dA_to_power_W(max_charge_current_dA, voltage_dV);

  datalayer_battery->status.max_discharge_power_W = current_dA_to_power_W(max_discharge_current_dA, voltage_dV);

  if (total_capacity_Wh > 0) {
    //real Data from Dyness battery
    datalayer_battery->info.total_capacity_Wh = total_capacity_Wh;
  }

  if (remaining_capacity_Wh > 0) {
    //real Data from Dyness battery
    datalayer_battery->status.remaining_capacity_Wh = remaining_capacity_Wh;
  } else {
    datalayer_battery->status.remaining_capacity_Wh = static_cast<uint32_t>(
        (static_cast<double>(datalayer_battery->status.real_soc) / 10000) * datalayer_battery->info.total_capacity_Wh);
  }
  // Update cell count if we've received individual cell data
  if (actual_cell_count > 0) {
    datalayer_battery->info.number_of_cells = actual_cell_count;
  }

  // Use Pylon / Dyness protocol min/max for alarms
  datalayer_battery->status.cell_max_voltage_mV = cellvoltage_max_mV;
  datalayer_battery->status.cell_min_voltage_mV = cellvoltage_min_mV;

  // Only populate cell voltages if we don't have individual cell data
  if (actual_cell_count == 0) {
    datalayer_battery->status.cell_voltages_mV[0] = cellvoltage_max_mV;
    datalayer_battery->status.cell_voltages_mV[1] = cellvoltage_min_mV;
  }

  datalayer_battery->status.temperature_min_dC = celltemperature_min_dC;

  datalayer_battery->status.temperature_max_dC = celltemperature_max_dC;

  // Expose the BMS-announced cutoffs on the battery info page
  extended_data.charge_cutoff_dV = charge_cutoff_voltage;
  extended_data.discharge_cutoff_dV = discharge_cutoff_voltage;

  // BMS cutoffs only define the design voltage when the user has not set explicit limits
  if (user_selected_max_pack_voltage_dV == 0) {
    datalayer_battery->info.max_design_voltage_dV = charge_cutoff_voltage;
  }
  if (user_selected_min_pack_voltage_dV == 0) {
    datalayer_battery->info.min_design_voltage_dV = discharge_cutoff_voltage;
  }

  /* Sleep/Wake mode: a BMS we put to sleep on purpose may stop sending. safety.cpp counts
     CAN_battery_still_alive down once per second and raises EVENT_CAN_BATTERY_MISSING at 0, so it is
     held up while the sleep is commanded. Only after the BMS has been heard once, so a BMS that never
     answered is still reported. Once wake is requested the hold stops and the normal 60 s check runs. */
  if (user_selected_pylon_relay_control == RELAY_MODE_SLEEP && !extended_data.want_closed && bms_frame_seen) {
    datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  }

  update_dc_bus_live(millis());
}

void PylonBattery::handle_incoming_can_frame(CAN_frame rx_frame) {

  switch (rx_frame.ID) {
    case EMUS_BASE_ID: {
      // EMUS configuration frame containing cell count
      uint8_t cell_count = rx_frame.data.u8[7];
      if (cell_count > 0 && cell_count <= MAX_CELLS) {
        actual_cell_count = cell_count;
      }
    } break;
    case 0x7310:  //System equipment info
    case 0x7311:
      //not sure if this is correct for Dyness Batteries
      hardware_version = rx_frame.data.u8[0];
      hardware_version_V = rx_frame.data.u8[2];
      hardware_version_R = rx_frame.data.u8[3];
      software_version_major = rx_frame.data.u8[4];
      software_version_minor = rx_frame.data.u8[5];
      break;
    case 0x7320:
    case 0x7321:
      battery_module_quantity = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]);
      //For Dyness Batteries
      if (actual_cell_count == 0) {
        actual_cell_count = battery_module_quantity;
      }
      battery_modules_in_series = rx_frame.data.u8[2];
      cell_quantity_in_module = rx_frame.data.u8[3];
      voltage_level = ((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]);
      ah_number = ((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]);
      break;
    case 0x7330:
      manufacturer_name[0] = rx_frame.data.u8[0];
      manufacturer_name[1] = rx_frame.data.u8[1];
      manufacturer_name[2] = rx_frame.data.u8[2];
      manufacturer_name[3] = rx_frame.data.u8[3];
      manufacturer_name[4] = rx_frame.data.u8[4];
      manufacturer_name[5] = rx_frame.data.u8[5];
      manufacturer_name[6] = rx_frame.data.u8[6];
      manufacturer_name[7] = rx_frame.data.u8[7];
      break;
    case 0x7340:
      manufacturer_name[8] = rx_frame.data.u8[0];
      manufacturer_name[9] = rx_frame.data.u8[1];
      manufacturer_name[10] = rx_frame.data.u8[2];
      manufacturer_name[11] = rx_frame.data.u8[3];
      manufacturer_name[12] = rx_frame.data.u8[4];
      manufacturer_name[13] = rx_frame.data.u8[5];
      manufacturer_name[14] = rx_frame.data.u8[6];
      manufacturer_name[15] = rx_frame.data.u8[7];
      break;
    case 0x42B0 ... 0x42EF:  // V1.27+ replacements, 0x42B0+A ... 0x42E0+A
      if (user_selected_pylon_relay_control == RELAY_MODE_OFF) {
        break;  // Relay control Off keeps the original parser, which never read these
      }
      [[fallthrough]];
    case 0x4210 ... 0x429F:  // Ensemble info, 0x4210+A ... 0x4290+A
      if (accept_address(rx_frame.ID & 0x000F)) {
        handle_ensemble_frame(rx_frame.ID & 0xFFF0, rx_frame);
      }
      break;
    case 0x8230 ... 0x823F:  // Answer to the forced relay command 0x8220+A
      // Logged only when it changes, the BMS may answer every second
      if (!extended_data.have_8230_response || memcmp(extended_data.last_8230_data, rx_frame.data.u8, 8) != 0) {
        logging.printf("[Pylon] 0x%04X relay answer: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                       (unsigned int)rx_frame.ID, rx_frame.data.u8[0], rx_frame.data.u8[1], rx_frame.data.u8[2],
                       rx_frame.data.u8[3], rx_frame.data.u8[4], rx_frame.data.u8[5], rx_frame.data.u8[6],
                       rx_frame.data.u8[7]);
      }
      extended_data.have_8230_response = true;
      extended_data.last_8230_response_millis = millis();
      memcpy(extended_data.last_8230_data, rx_frame.data.u8, 8);
      break;
    case 0x18FF9A6E:
      if (rx_frame.data.u8[0] == 0x03) {
        remaining_capacity_Wh = ((rx_frame.data.u8[4] << 8) | rx_frame.data.u8[3]) * 10;
        total_capacity_Wh = ((rx_frame.data.u8[6] << 8) | rx_frame.data.u8[5]) * 10;
      }
      break;
    case DYNESS_CELL_VOLTAGE_BASE_ID: {
      // Handle EMUS individual cell voltage
      uint8_t cell_start_voltage = rx_frame.data.u8[0] - 1;
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;  // Keep battery alive on cell data
      //three cells per message
      for (uint8_t i = 0; i < 3; i++) {
        uint8_t cell_index_voltage = cell_start_voltage + i;
        if (cell_index_voltage < MAX_CELLS && (actual_cell_count == 0 || cell_index_voltage < actual_cell_count)) {
          //start with the second byte and advance by 2 bytes for each cell (3 cells per message)
          int16_t cell_voltage = (int8_t)rx_frame.data.u8[(i * 2) + 1];
          // int8_t is correct between 2880mV and 3520mV, but above 3520mV the byte will overflow and will show the wrong value.
          // To avoid the overflow at 3520mV, we look at the minimum cell voltage and if it is above 3200mV, we use the unsigned value instead of the signed value
          if (cellvoltage_min_mV > 3200) {
            //Most likely there is a more elegant way to do this, but this works for now and ensures we get accurate voltages
            cell_voltage = (uint8_t)rx_frame.data.u8[(i * 2) + 1];
          }

          // Cell voltage: 3200mV base + (byte value × 2.5mV)
          datalayer_battery->status.cell_voltages_mV[cell_index_voltage] = 3200 + cell_voltage * 2.5;
        }
      }
    } break;
    case PYLON_CELL_VOLTAGE_BASE_ID ...(PYLON_CELL_VOLTAGE_BASE_ID + 31): {
      // Handle EMUS individual cell voltage messages (0x19B50100-0x19B5011F)
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;  // Keep battery alive on cell data
      uint8_t group = rx_frame.ID - PYLON_CELL_VOLTAGE_BASE_ID;
      uint8_t cell_start = group * 8;

      for (uint8_t i = 0; i < 8; i++) {
        uint8_t cell_index = cell_start + i;
        if (cell_index < MAX_CELLS && (actual_cell_count == 0 || cell_index < actual_cell_count)) {
          // Cell voltage: 2000mV base + (byte value × 10mV)
          uint16_t cell_voltage = 2000 + (rx_frame.data.u8[i] * 10);
          // Only update if voltage is in valid range (2500-4200mV)
          if (cell_voltage >= 2500 && cell_voltage <= 4200) {
            uint16_t current_voltage = datalayer_battery->status.cell_voltages_mV[cell_index];
            // Reject sudden large changes (>1000mV)
            if (current_voltage == 0 || abs((int)cell_voltage - (int)current_voltage) <= 1000) {
              datalayer_battery->status.cell_voltages_mV[cell_index] = cell_voltage;
            }
          }
        }
      }
    } break;
    case PYLON_CELL_BALANCING_BASE_ID ...(PYLON_CELL_BALANCING_BASE_ID + 31): {
      // Handle EMUS individual cell balancing status messages (0x19B50300-0x19B5031F)
      uint8_t group = rx_frame.ID - PYLON_CELL_BALANCING_BASE_ID;
      uint8_t cell_start = group * 8;

      for (uint8_t i = 0; i < 8; i++) {
        uint8_t cell_index = cell_start + i;
        if (cell_index < MAX_CELLS && (actual_cell_count == 0 || cell_index < actual_cell_count)) {
          datalayer_battery->status.cell_balancing_status.set(cell_index, rx_frame.data.u8[i] > 0);
        }
      }
    } break;
      //default:
      //nothing to do
  }
}

bool PylonBattery::accept_address(uint8_t address) {
  if (user_selected_pylon_bms_address != 0) {
    return address == user_selected_pylon_bms_address;
  }
  if (user_selected_pylon_relay_control == RELAY_MODE_OFF) {
    return address <= 1;  // What the driver always accepted: 0x42x0 and 0x42x1
  }
  // Auto: follow the first BMS heard, so a second pack on the bus cannot mix its values in
  return locked_rx_address < 0 || address == locked_rx_address;
}

static inline int32_t le_int32(const CAN_frame& frame, uint8_t index) {
  return (int32_t)((uint32_t)frame.data.u8[index] | ((uint32_t)frame.data.u8[index + 1] << 8) |
                   ((uint32_t)frame.data.u8[index + 2] << 16) | ((uint32_t)frame.data.u8[index + 3] << 24));
}

static inline int16_t clamp_int16(int32_t value, int32_t low) {
  return (int16_t)(value < low ? low : (value > INT16_MAX ? INT16_MAX : value));
}

void PylonBattery::handle_ensemble_frame(uint32_t family, const CAN_frame& rx_frame) {
  const uint8_t address = rx_frame.ID & 0x000F;
  if (locked_rx_address < 0) {
    locked_rx_address = address;
    extended_data.bms_address_detected = address == 0 ? 1 : address;
    logging.printf("[Pylon] BMS heard on address %u (0x%04X)\n", address, (unsigned int)rx_frame.ID);
  }
  last_bms_frame_ms = millis();
  bms_frame_seen = true;

  switch (family) {
    case 0x4210:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      extended_data.old_frames_seen = true;
      old_frame_current_dA = ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]) - 30000;
      if (!new_voltage_frame_seen) {
        voltage_dV = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]);
        current_dA = old_frame_current_dA;
        SOC = rx_frame.data.u8[6];
        SOH = rx_frame.data.u8[7];
      }
      if (!new_cutoff_frame_seen) {
        BMS_temperature_dC = (((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4])) - 1000;
      }
      break;
    case 0x4220:
      if (!new_cutoff_frame_seen) {
        charge_cutoff_voltage = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]);
        discharge_cutoff_voltage = ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]);
      }
      if (!new_current_frame_seen) {
        max_charge_current_dA = ((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]) - 30000;
        // Some BMS (e.g. GCE) encode the discharge limit with a positive offset like the
        // charge limit, instead of the standard negative offset. abs() handles both.
        max_discharge_current_dA = abs((((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]) - 30000));
      }
      break;
    case 0x4230:
      cellvoltage_max_mV = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]);
      cellvoltage_min_mV = ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]);
      extended_data.cell_max_number = ((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]);
      extended_data.cell_min_number = ((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]);
      break;
    case 0x4240:
      celltemperature_max_dC = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]) - 1000;
      celltemperature_min_dC = ((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[2]) - 1000;
      extended_data.temp_max_sensor = ((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]);
      extended_data.temp_min_sensor = ((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]);
      break;
    case 0x4250: {
      //Byte1-2 Cycle Period
      const uint8_t raw = rx_frame.data.u8[0];
      const uint8_t fault = rx_frame.data.u8[3];
      const uint16_t alarm = (rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4];
      const uint16_t protection = (rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6];
      if (!extended_data.have_4250 || raw != extended_data.basic_status_raw || fault != extended_data.fault ||
          alarm != extended_data.alarm || protection != extended_data.protection) {
        // Every change is logged: which bits follow the relay is not known for every BMS
        logging.printf("[Pylon] 0x4250: status 0x%02X fault 0x%02X alarm 0x%04X protection 0x%04X\n", raw, fault, alarm,
                       protection);
      }
      extended_data.have_4250 = true;
      extended_data.basic_status = raw & 0x07;
      extended_data.basic_status_raw = raw;
      extended_data.fault = fault;
      extended_data.alarm = alarm;
      extended_data.protection = protection;
    } break;
    case 0x4260:
      //Byte0-1 Module Max Voltage
      //Byte2-3 Module Min Voltage
      //Byte4-5 Module Max. Voltage Number
      //Byte6-7 Module Min. Voltage Number
      break;
    case 0x4270:
      //Byte0-1 Module Max. Temperature
      //Byte2-3 Module Min. Temperature
      //Byte4-5 Module Max. Temperature Number
      //Byte6-7 Module Min. Temperature Number
      break;
    case 0x4280:
      if (rx_frame.data.u8[0] != charge_forbidden || rx_frame.data.u8[1] != discharge_forbidden) {
        logging.printf("[Pylon] 0x4280: charge forbidden 0x%02X discharge forbidden 0x%02X\n", rx_frame.data.u8[0],
                       rx_frame.data.u8[1]);
      }
      charge_forbidden = rx_frame.data.u8[0];
      discharge_forbidden = rx_frame.data.u8[1];
      extended_data.charge_forbidden = charge_forbidden;
      extended_data.discharge_forbidden = discharge_forbidden;
      break;
    case 0x42B0: {
      // Same sign convention as 0x4210 (raw - 30000), just without the offset
      const int32_t new_current_dA = le_int32(rx_frame, 2);
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      if (!current_sign_checked && extended_data.old_frames_seen) {
        current_sign_checked = true;
        const bool opposite =
            (old_frame_current_dA > 20 && new_current_dA < -20) || (old_frame_current_dA < -20 && new_current_dA > 20);
        logging.printf("[Pylon] Current check: 0x4210 %d dA, 0x42B0 %ld dA%s\n", old_frame_current_dA,
                       (long)new_current_dA, opposite ? " - SIGN MISMATCH" : "");
      }
      new_voltage_frame_seen = true;
      extended_data.new_frames_seen = true;
      voltage_dV = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]);
      current_dA = clamp_int16(new_current_dA, INT16_MIN);
      SOC = rx_frame.data.u8[6];
      SOH = rx_frame.data.u8[7];
    } break;
    case 0x42C0:
      new_cutoff_frame_seen = true;
      extended_data.new_frames_seen = true;
      BMS_temperature_dC = (((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0])) - 1000;
      charge_cutoff_voltage = ((rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]);
      discharge_cutoff_voltage = ((rx_frame.data.u8[7] << 8) | rx_frame.data.u8[6]);
      break;
    case 0x42D0:
      new_current_frame_seen = true;
      extended_data.new_frames_seen = true;
      max_charge_current_dA = clamp_int16(le_int32(rx_frame, 0), 0);
      max_discharge_current_dA = clamp_int16(abs(le_int32(rx_frame, 4)), 0);
      break;
    default:  // 0x4290 and 0x42E0 are not used
      break;
  }
}

// Relay modes 1 and 2 only. Mode 0 is sent by transmit_can() exactly as before.
void PylonBattery::send_relay_commands(bool want_closed) {
  const uint8_t address = extended_data.command_address;

  PYLON_8200.ID = 0x8200 + address;
  if (user_selected_pylon_relay_control == RELAY_MODE_SLEEP) {
    PYLON_8200.data.u8[0] = want_closed ? 0xAA : 0x55;  // AA = Quit sleep, 55 = Goto sleep
  } else {
    PYLON_8200.data.u8[0] = 0xAA;
  }
  transmit_can_frame(&PYLON_8200);

  if (user_selected_pylon_relay_control == RELAY_MODE_FORCE) {
    PYLON_8220.ID = 0x8220 + address;
    PYLON_8220.data.u8[0] = want_closed ? 0x55 : 0xAA;  // 55 = closing allowed again, AA = force open
    transmit_can_frame(&PYLON_8220);
  }

  PYLON_8210.ID = 0x8210 + address;
  PYLON_8210.data.u8[0] = want_closed ? 0xAA : 0x00;  // Charge command
  PYLON_8210.data.u8[1] = 0x00;                       // Discharge command
  transmit_can_frame(&PYLON_8210);
  // 0x8240 (mask external comm loss) and 0x8260 (Run) are never sent: they would bypass BMS protections
}

/* Modes 1 and 2 only. The Solax driver reports dc_bus_live to the inverter as the contactor state
   (SOLAX_1875 byte 4) and its bus_watch walks BUS_WAIT_LIVE -> BUS_LIVE -> BUS_DROPPED -> live again,
   replaying the announce when the pack comes back. So dc_bus_live has to follow the real JBD relay:
   false at once when opening is requested, true only after the close delay while the BMS is talking,
   awake and not reporting RELAY_ERR. Left alone with GPIO contactor control, which owns the flag then,
   and by a Pylon running as 2nd/3rd battery, as the flag belongs to the main battery. */
bool PylonBattery::owns_dc_bus_live() {
  return user_selected_pylon_relay_control != RELAY_MODE_OFF && !contactor_control_enabled &&
         allows_contactor_closing != nullptr;
}

void PylonBattery::update_dc_bus_live(unsigned long currentMillis) {
  if (!owns_dc_bus_live()) {
    return;
  }
  const bool want_closed = extended_data.want_closed;
  const bool bms_talking = bms_frame_seen && (currentMillis - last_bms_frame_ms) < BMS_SILENT_AFTER_MS;
  const bool relay_error = (extended_data.fault & FAULT_RELAY_ERR) != 0;
  const bool bms_asleep = extended_data.have_4250 && extended_data.basic_status == 0;
  const bool delay_done = (currentMillis - want_closed_since_ms) >= user_selected_pylon_relay_close_delay_ms;

  const bool live = want_closed && delay_done && bms_talking && !relay_error && !bms_asleep;
  if (live != datalayer.system.status.dc_bus_live) {
    logging.printf("[Pylon] DC bus %s (want_closed %d, talking %d, relay_err %d, asleep %d)\n", live ? "live" : "dead",
                   want_closed, bms_talking, relay_error, bms_asleep);
  }
  datalayer.system.status.dc_bus_live = live;
}

void PylonBattery::transmit_can(unsigned long currentMillis) {
  // Bench debugging (PYLON-DEBUG.h): frames queued from the web or USB console go out from here
  pylon_debug.service(currentMillis, [this](const CAN_frame* frame) { transmit_can_frame(frame); });
  if (pylon_debug.mute == PylonDebug::MUTE_ALL) {
    return;
  }
  const bool relay_muted = pylon_debug.mute == PylonDebug::MUTE_RELAY;

  // The debug console can change the mode at runtime; start over as after a boot
  if (user_selected_pylon_relay_control != active_relay_mode) {
    if (active_relay_mode != 0xFF) {
      logging.printf("[Pylon] Relay control mode %u -> %u\n", active_relay_mode, user_selected_pylon_relay_control);
      if (user_selected_pylon_relay_control == RELAY_MODE_OFF && allows_contactor_closing != nullptr &&
          !contactor_control_enabled) {
        datalayer.system.status.dc_bus_live = true;  // Mode 0 leaves the flag at its default, as originally
      }
    }
    active_relay_mode = user_selected_pylon_relay_control;
    relay_commands_sent = false;
  }

  const bool relay_control = user_selected_pylon_relay_control != RELAY_MODE_OFF;
  const bool want_closed =
      datalayer.system.status.inverter_allows_contactor_closing && !datalayer.system.info.equipment_stop_active;

  extended_data.relay_control_mode = user_selected_pylon_relay_control;
  extended_data.address_is_auto = user_selected_pylon_bms_address == 0;
  extended_data.command_address = user_selected_pylon_bms_address != 0      ? user_selected_pylon_bms_address
                                  : extended_data.bms_address_detected != 0 ? extended_data.bms_address_detected
                                                                            : 1;
  extended_data.want_closed = want_closed;

  if (want_closed != previous_want_closed || !relay_commands_sent) {
    if (want_closed) {
      want_closed_since_ms = currentMillis;
    }
    if (relay_control) {
      logging.printf("[Pylon] Relay %s requested, BMS address %u%s\n", want_closed ? "close" : "open",
                     extended_data.command_address, relay_muted ? " (muted by debug)" : "");
      if (!relay_muted) {
        send_relay_commands(want_closed);  // Right away, not only on the next 1 s tick
      }
      if (!want_closed && owns_dc_bus_live()) {
        datalayer.system.status.dc_bus_live = false;
      }
    }
    previous_want_closed = want_closed;
    relay_commands_sent = true;
  }

  // Send 1s CAN Message
  if (currentMillis - previousMillis1000 >= INTERVAL_1_S) {
    previousMillis1000 = currentMillis;

    transmit_can_frame(&PYLON_3010);  // Heartbeat
    transmit_can_frame(&PYLON_4200);  // Ensemble OR System equipment info, depends on frame0

    if (relay_muted) {
      // Debug: only the frames queued from the console control the relay
    } else if (relay_control) {
      send_relay_commands(want_closed);
    } else {
      // Relay control Off: the original messages, unaddressed, no 0x8220
      PYLON_8200.data.u8[0] = 0xAA;  //AA = Quit sleep, 55 = Goto sleep

      PYLON_8210.data.u8[0] = 0xAA;
      /*Charge Command: When the battery is in under-voltage protection, the contactors are open. When
      we are about to charge the battery, send this command, then the battery will close contactors.
      If the battery is in sleep status, wake up first then use this command.*/

      PYLON_8210.data.u8[1] = 0x00;
      /*Discharge Command: When the battery is in over-voltage protection, the contactors are open. When
      we are about to discharge the battery, send this command, then the battery will close contactors.
      If the battery is in sleep status, wake up first then use this command.*/

      transmit_can_frame(&PYLON_8200);  // Control device quit sleep status
      transmit_can_frame(&PYLON_8210);  // Charge command
    }

    mux = (mux + 1) % 3;  // mux cycles between 0-1-2-0-1...
    PYLON_4200.data.u8[0] = mux;
    /*00 Request Ensamble Information (Battery will respond 0x42XX messages)
    01 Request Cellvoltages (Battery will respond 0x5XXX messages)
    02 Request System equipment info (Battery will respond 0x73XX messages)*/
  }

  // Poll for individual cell voltages every 5 seconds
  if (currentMillis - previousMillis5000 >= INTERVAL_5_S) {
    previousMillis5000 = currentMillis;

    // Request cell voltage data from EMUS BMS
    transmit_can_frame(&EMUS_CELL_VOLTAGE_REQUEST);
    // Request cell balancing status from EMUS BMS
    transmit_can_frame(&EMUS_CELL_BALANCING_REQUEST);
  }
}

void PylonBattery::setup(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.battery_protocol, "Pylon / Dyness compatible battery", 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer_battery->info.number_of_cells = 2;
  if (user_selected_max_pack_voltage_dV > 0) {
    datalayer_battery->info.max_design_voltage_dV = user_selected_max_pack_voltage_dV;
  }
  if (user_selected_min_pack_voltage_dV > 0) {
    datalayer_battery->info.min_design_voltage_dV = user_selected_min_pack_voltage_dV;
  }
  if (user_selected_max_cell_voltage_mV > 0) {
    datalayer_battery->info.max_cell_voltage_mV = user_selected_max_cell_voltage_mV;
  }
  if (user_selected_min_cell_voltage_mV > 0) {
    datalayer_battery->info.min_cell_voltage_mV = user_selected_min_cell_voltage_mV;
  }
  datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;

  // The inverter decides when to close; the relay itself is driven over CAN in modes 1 and 2
  if (allows_contactor_closing) {
    *allows_contactor_closing = true;
  }

  if (owns_dc_bus_live()) {
    datalayer.system.status.dc_bus_live = false;  // Until the BMS has been heard and the close delay is over
  }
}

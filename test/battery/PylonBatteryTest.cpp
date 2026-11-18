#include <gtest/gtest.h>

#include "../../Software/src/battery/PYLON-BATTERY.h"
#include "../../Software/src/datalayer/datalayer.h"

#include "Arduino.h"

// Pin the current-limit -> power-limit conversion in the Pylon/Dyness driver.

namespace {

// Little-endian 16-bit word into bytes[index], bytes[index+1].
void put_le16(CAN_frame& frame, uint8_t index, uint16_t value) {
  frame.data.u8[index] = value & 0xFF;
  frame.data.u8[index + 1] = (value >> 8) & 0xFF;
}

// 0x4210:
// voltage_dV @0
// current word (dA + 30000) @2
// temp word (dC + 1000) @4
// SOC @6
// SOH @7
CAN_frame pylon_4210(uint16_t voltage_dV, int16_t current_dA, uint8_t soc, uint8_t soh) {
  CAN_frame frame = {};
  frame.ext_ID = true;
  frame.DLC = 8;
  frame.ID = 0x4210;
  put_le16(frame, 0, voltage_dV);
  put_le16(frame, 2, static_cast<uint16_t>(current_dA + 30000));
  put_le16(frame, 4, static_cast<uint16_t>(1000));  // 0.0 degC
  frame.data.u8[6] = soc;
  frame.data.u8[7] = soh;
  return frame;
}

// 0x4220:
// charge cutoff @0
// discharge cutoff @2
// max charge current (dA + 30000) @4
// max discharge current (dA + 30000) @6
CAN_frame pylon_4220(int16_t max_charge_current_dA, int16_t max_discharge_current_dA) {
  CAN_frame frame = {};
  frame.ext_ID = true;
  frame.DLC = 8;
  frame.ID = 0x4220;
  put_le16(frame, 0, 4088);  // charge cutoff voltage, unused by these assertions
  put_le16(frame, 2, 2968);  // discharge cutoff voltage, unused
  put_le16(frame, 4, static_cast<uint16_t>(max_charge_current_dA + 30000));
  put_le16(frame, 6, static_cast<uint16_t>(max_discharge_current_dA + 30000));
  return frame;
}

// Feeds voltage + the given current limits through the real decode path and publishes to datalayer.
void decode_limits(PylonBattery* battery, uint16_t voltage_dV, int16_t max_charge_current_dA,
                   int16_t max_discharge_current_dA) {
  battery->handle_incoming_can_frame(pylon_4210(voltage_dV, 0, 50, 100));
  battery->handle_incoming_can_frame(pylon_4220(max_charge_current_dA, max_discharge_current_dA));
  battery->update_values();
}

}  // namespace

// 0.1 A limit on a 360 V pack: 1 dA * 3600 dV / 100 = 36 W
TEST(PylonBatteryTests, SubOneAmpLimitDoesNotTruncateToZero) {
  auto battery = new PylonBattery();
  decode_limits(battery, 3600, 1, 1);

  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 36u);
  EXPECT_EQ(datalayer.battery.status.max_discharge_power_W, 36u);

  delete battery;
}

// A fractional-amp mid-range limit must keep the 0.5 A: 155 dA * 3600 dV / 100 = 5580 W
TEST(PylonBatteryTests, FractionalAmpLimitIsNotDiscarded) {
  auto battery = new PylonBattery();
  decode_limits(battery, 3600, 155, 155);

  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 5580u);
  EXPECT_EQ(datalayer.battery.status.max_discharge_power_W, 5580u);

  delete battery;
}

// ---- BMS address, V1.27+ frames and relay control ----

#include <vector>
#include "../../Software/src/communication/contactorcontrol/comm_contactorcontrol.h"

void clear_transmitted_frames();
const std::vector<CAN_frame>& get_transmitted_frames();

namespace {

void put_le32(CAN_frame& frame, uint8_t index, int32_t value) {
  for (uint8_t i = 0; i < 4; i++) {
    frame.data.u8[index + i] = (static_cast<uint32_t>(value) >> (8 * i)) & 0xFF;
  }
}

CAN_frame frame_with_id(uint32_t id) {
  CAN_frame frame = {};
  frame.ext_ID = true;
  frame.DLC = 8;
  frame.ID = id;
  return frame;
}

CAN_frame addressed_4210(uint8_t address, uint16_t voltage_dV) {
  CAN_frame frame = pylon_4210(voltage_dV, 0, 50, 100);
  frame.ID = 0x4210 + address;
  return frame;
}

CAN_frame pylon_4250(uint8_t address, uint8_t basic_status, uint8_t fault) {
  CAN_frame frame = frame_with_id(0x4250 + address);
  frame.data.u8[0] = basic_status;
  frame.data.u8[3] = fault;
  return frame;
}

const CAN_frame* find_frame(uint32_t id) {
  const CAN_frame* found = nullptr;
  for (const auto& frame : get_transmitted_frames()) {
    if (frame.ID == id) {
      found = &frame;  // Last one wins
    }
  }
  return found;
}

class PylonRelayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    user_selected_pylon_bms_address = 0;
    user_selected_pylon_relay_control = 0;
    user_selected_pylon_relay_close_delay_ms = 3000;
    contactor_control_enabled = false;
    datalayer.system.status.inverter_allows_contactor_closing = true;
    datalayer.system.info.equipment_stop_active = false;
    datalayer.system.status.dc_bus_live = true;
    datalayer.battery.status.voltage_dV = 0;
    set_millis64(100000);
    clear_transmitted_frames();
  }
  void TearDown() override {
    user_selected_pylon_bms_address = 0;
    user_selected_pylon_relay_control = 0;
    user_selected_pylon_relay_close_delay_ms = 3000;
    datalayer.system.info.equipment_stop_active = false;
    datalayer.system.status.dc_bus_live = true;
    clear_transmitted_frames();
  }
  void advance(uint32_t ms) { set_millis64(millis64() + ms); }
};

}  // namespace

TEST_F(PylonRelayTest, ModeOffSendsTheOriginalFrames) {
  PylonBattery battery;
  battery.setup();
  battery.transmit_can(millis());

  ASSERT_NE(find_frame(0x8200), nullptr);
  ASSERT_NE(find_frame(0x8210), nullptr);
  EXPECT_EQ(find_frame(0x8200)->data.u8[0], 0xAA);
  EXPECT_EQ(find_frame(0x8210)->data.u8[0], 0xAA);
  EXPECT_EQ(find_frame(0x8210)->data.u8[1], 0x00);
  for (const auto& frame : get_transmitted_frames()) {
    EXPECT_NE(frame.ID & 0xFFF0, 0x8220u);
  }
  EXPECT_TRUE(datalayer.system.status.dc_bus_live);
}

TEST_F(PylonRelayTest, ModeOffAutoAddressAcceptsOnlyAddressZeroAndOne) {
  PylonBattery battery;
  battery.handle_incoming_can_frame(addressed_4210(1, 3600));
  battery.update_values();
  EXPECT_EQ(datalayer.battery.status.voltage_dV, 3600);

  battery.handle_incoming_can_frame(addressed_4210(2, 3500));
  battery.update_values();
  EXPECT_EQ(datalayer.battery.status.voltage_dV, 3600);
}

TEST_F(PylonRelayTest, FixedAddressIgnoresOtherBms) {
  user_selected_pylon_bms_address = 5;
  PylonBattery battery;
  battery.handle_incoming_can_frame(addressed_4210(3, 3500));
  battery.handle_incoming_can_frame(addressed_4210(5, 3600));
  battery.handle_incoming_can_frame(addressed_4210(3, 3500));
  battery.update_values();
  EXPECT_EQ(datalayer.battery.status.voltage_dV, 3600);
}

TEST_F(PylonRelayTest, AutoAddressIsLearnedAndUsedForCommands) {
  user_selected_pylon_relay_control = 1;
  PylonBattery battery;
  battery.setup();
  battery.transmit_can(millis());
  EXPECT_NE(find_frame(0x8201), nullptr);  // Unknown address: commands go to 1

  battery.handle_incoming_can_frame(addressed_4210(3, 3600));
  battery.handle_incoming_can_frame(addressed_4210(4, 3000));  // Another BMS, ignored
  battery.update_values();
  EXPECT_EQ(datalayer.battery.status.voltage_dV, 3600);

  clear_transmitted_frames();
  advance(1000);
  battery.transmit_can(millis());
  EXPECT_NE(find_frame(0x8203), nullptr);
  EXPECT_NE(find_frame(0x8213), nullptr);
  ASSERT_NE(find_frame(0x8223), nullptr);
  EXPECT_EQ(find_frame(0x8223)->data.u8[0], 0x55);
}

TEST_F(PylonRelayTest, ForceRelayOpensImmediatelyAndClosesAfterDelay) {
  user_selected_pylon_relay_control = 1;
  user_selected_pylon_bms_address = 2;
  PylonBattery battery;
  battery.setup();
  battery.transmit_can(millis());
  battery.handle_incoming_can_frame(addressed_4210(2, 3600));
  battery.handle_incoming_can_frame(pylon_4250(2, 3, 0));

  advance(500);
  battery.update_values();
  EXPECT_FALSE(datalayer.system.status.dc_bus_live);  // Close delay not over yet

  advance(3000);
  battery.handle_incoming_can_frame(addressed_4210(2, 3600));
  battery.update_values();
  EXPECT_TRUE(datalayer.system.status.dc_bus_live);

  // Open Contactors: sent at once, without waiting for the 1 s tick
  clear_transmitted_frames();
  datalayer.system.info.equipment_stop_active = true;
  advance(10);
  battery.transmit_can(millis());
  ASSERT_NE(find_frame(0x8222), nullptr);
  ASSERT_NE(find_frame(0x8212), nullptr);
  ASSERT_NE(find_frame(0x8202), nullptr);
  EXPECT_EQ(find_frame(0x8222)->data.u8[0], 0xAA);
  EXPECT_EQ(find_frame(0x8212)->data.u8[0], 0x00);
  EXPECT_EQ(find_frame(0x8202)->data.u8[0], 0xAA);
  EXPECT_FALSE(datalayer.system.status.dc_bus_live);

  battery.handle_incoming_can_frame(frame_with_id(0x8232));  // Acknowledgement is accepted
}

TEST_F(PylonRelayTest, RelayErrorOrSilentBmsKeepsBusDead) {
  user_selected_pylon_relay_control = 1;
  user_selected_pylon_relay_close_delay_ms = 0;
  PylonBattery battery;
  battery.setup();
  battery.transmit_can(millis());
  battery.handle_incoming_can_frame(addressed_4210(1, 3600));
  battery.handle_incoming_can_frame(pylon_4250(1, 3, 0x20));
  battery.update_values();
  EXPECT_FALSE(datalayer.system.status.dc_bus_live);

  battery.handle_incoming_can_frame(pylon_4250(1, 3, 0x00));
  battery.update_values();
  EXPECT_TRUE(datalayer.system.status.dc_bus_live);

  advance(6000);  // BMS stopped talking
  battery.update_values();
  EXPECT_FALSE(datalayer.system.status.dc_bus_live);
}

TEST_F(PylonRelayTest, SleepWakeSendsSleepAndHoldsAliveCounter) {
  user_selected_pylon_relay_control = 2;
  PylonBattery battery;
  battery.setup();
  battery.transmit_can(millis());
  battery.handle_incoming_can_frame(addressed_4210(1, 3600));

  clear_transmitted_frames();
  datalayer.system.info.equipment_stop_active = true;
  battery.transmit_can(millis());
  ASSERT_NE(find_frame(0x8201), nullptr);
  EXPECT_EQ(find_frame(0x8201)->data.u8[0], 0x55);
  EXPECT_EQ(find_frame(0x8221), nullptr);

  datalayer.battery.status.CAN_battery_still_alive = 0;  // BMS asleep and silent
  battery.update_values();
  EXPECT_EQ(datalayer.battery.status.CAN_battery_still_alive, CAN_STILL_ALIVE);

  datalayer.system.info.equipment_stop_active = false;  // Wake: normal check again
  battery.transmit_can(millis());
  datalayer.battery.status.CAN_battery_still_alive = 0;
  battery.update_values();
  EXPECT_EQ(datalayer.battery.status.CAN_battery_still_alive, 0);
}

TEST_F(PylonRelayTest, ModeOffIgnoresNewFrames) {
  PylonBattery battery;
  CAN_frame b0 = frame_with_id(0x42B1);
  put_le16(b0, 0, 6200);
  battery.handle_incoming_can_frame(pylon_4210(6100, 0, 50, 100));
  battery.handle_incoming_can_frame(b0);
  battery.update_values();
  EXPECT_EQ(datalayer.battery.status.voltage_dV, 6100);
}

TEST_F(PylonRelayTest, NewFramesWinOverOldOnes) {
  user_selected_pylon_relay_control = 1;
  PylonBattery battery;
  CAN_frame b0 = frame_with_id(0x42B0);
  put_le16(b0, 0, 6200);
  put_le32(b0, 2, -1234);  // Discharging 123.4 A, signed, no offset
  b0.data.u8[6] = 77;
  b0.data.u8[7] = 99;
  battery.handle_incoming_can_frame(pylon_4210(6100, 500, 50, 100));
  battery.handle_incoming_can_frame(b0);
  battery.handle_incoming_can_frame(pylon_4210(6100, 500, 50, 100));  // Old frame no longer wins

  CAN_frame d0 = frame_with_id(0x42D0);
  put_le32(d0, 0, 1000);
  put_le32(d0, 4, 1500);
  battery.handle_incoming_can_frame(d0);
  battery.handle_incoming_can_frame(pylon_4220(10, 10));
  battery.update_values();

  EXPECT_EQ(datalayer.battery.status.voltage_dV, 6200);
  EXPECT_EQ(datalayer.battery.status.current_dA, -1234);
  EXPECT_EQ(datalayer.battery.status.real_soc, 7700);
  EXPECT_EQ(datalayer.battery.status.max_charge_power_W, 1000u * 6200 / 100);
  EXPECT_EQ(datalayer.battery.status.max_discharge_power_W, 1500u * 6200 / 100);
}

#include "ovms_log.h"
static const char *TAG = "v-maxt90";

#include <stdio.h>
#include <string>
#include "vehicle_maxt90.h"
#include "vehicle_obdii.h"
#include "metrics_standard.h"
#include "ovms_metrics.h"

OvmsVehicleMaxt90::OvmsVehicleMaxt90()
{
  ESP_LOGI(TAG, "Initialising Maxus T90 EV vehicle module (derived from OBDII)");

  // Register CAN1 bus at 500 kbps
  RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);

  // Custom metrics:
  // Prefix "xmt" = Maxus T90 (to match xnl, xmg, etc. in other vehicles)
  // NOTE: adjust Celcius / kWh to match your metric_unit_t enum names
  m_hvac_temp_c =
    MyMetrics.InitFloat("xmt.v.hvac.temp", 10, 0.0f, Celcius, false);
  m_pack_capacity_kwh =
    MyMetrics.InitFloat("xmt.b.capacity", 0, 88.5f, kWh, true);
  m_dcdc_voltage =
    MyMetrics.InitFloat("xmt.v.dcdc.voltage", 120, 0.0f, Volts, false);

  // BMS cell monitor. The BMS reports 110 cell voltages (LFP, ~3.31 V each)
  // and 3 temperature sensors; the arrangement follows what 0xB142 returns.
  BmsSetCellArrangementVoltage(110, 10);
  BmsSetCellArrangementTemperature(3, 1);
  BmsSetCellLimitsVoltage(2.0, 4.5);
  BmsSetCellLimitsTemperature(-39, 200);
  BmsSetCellDefaultThresholdsVoltage(0.020, 0.030);
  BmsSetCellDefaultThresholdsTemperature(2.0, 3.0);

  // Define poll list:
  //  - State 0: vehicle off
  //  - State 1: vehicle on / driving (detected from the 0x266 broadcast)
  //  - State 2: charging (worked out in Ticker1, see the comment there)
  //
  // Only the plug detect (0xE009) is polled in state 0, so we don't keep the
  // ECUs awake. The state-2 columns keep SOC, SOH and the temperatures
  // updating while the car charges.
  static const OvmsPoller::poll_pid_t maxt90_polls[] = {
    // VIN (only when on / charge, slow rate)
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xF190,
      { 0, 3600, 3600 }, 0, ISOTP_STD },

    // SOC
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE002,
      { 0, 10, 10 }, 0, ISOTP_STD },

    // SOH
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE003,
      { 0, 1800, 1800 }, 0, ISOTP_STD },

    // Unknown drive value, NOT a READY flag (see the handler comment).
    // Kept polled while the car is active to collect data for decoding.
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE004,
      { 0, 10, 10 }, 0, ISOTP_STD },

    // Plug present. Polled in state 0 too, so the plug state is fresh by
    // the time the charge detection in Ticker1 needs it.
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE009,
      { 5, 10, 10 }, 0, ISOTP_STD },

    // HVAC temp
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE010,
      { 0, 30, 30 }, 0, ISOTP_STD },

    // Ambient temp
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE025,
      { 0, 30, 30 }, 0, ISOTP_STD },

    // AC charge current in whole amps, read from the on-board charger
    // (0x722). Only polled while charging.
    { 0x722, 0x7a2, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE001,
      { 0, 0, 10 }, 0, ISOTP_STD },

    // 12V system current (VCU)
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE022,
      { 0, 30, 30 }, 0, ISOTP_STD },

    // BMS (0x748/0x7c8) battery data. The BMS uses a 0xB1xx DID map,
    // not the 0xE0xx map the eDeliver3 uses.
    // Pack voltage
    { 0x748, 0x7c8, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xB105,
      { 0, 10, 10 }, 0, ISOTP_STD },
    // Cell voltage max / min / average
    { 0x748, 0x7c8, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xB114,
      { 0, 30, 30 }, 0, ISOTP_STD },
    // Pack temperature sensors (3)
    { 0x748, 0x7c8, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xB110,
      { 0, 30, 30 }, 0, ISOTP_STD },
    // 12V DC-DC output voltage
    { 0x748, 0x7c8, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xB136,
      { 0, 60, 60 }, 0, ISOTP_STD },
    // Full cell voltage array (multi-frame reply)
    { 0x748, 0x7c8, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xB142,
      { 0, 60, 60 }, 0, ISOTP_STD },

    POLL_LIST_END
  };

  // Attach the poll list to CAN1 & start in "off" state
  PollSetPidList(m_can1, maxt90_polls);
  PollSetState(0);

  ESP_LOGI(TAG, "Maxus T90 EV poller configured on CAN1 @ 500 kbps");
}

OvmsVehicleMaxt90::~OvmsVehicleMaxt90()
{
  ESP_LOGI(TAG, "Shutdown Maxus T90 EV vehicle module");
}

// ─────────────────────────────────────────────
//  Live CAN Frame Handler (Lock + Odometer)
// ─────────────────────────────────────────────
void OvmsVehicleMaxt90::IncomingFrameCan1(CAN_frame_t* p_frame)
{
  // Let the base class see the frame as well (for diagnostics etc.)
  OvmsVehicleOBDII::IncomingFrameCan1(p_frame);

  if (p_frame->origin != m_can1)
    return;

  const uint8_t* d = p_frame->data.u8;

  switch (p_frame->MsgID)
  {
    case 0x266: // Powertrain broadcast, only sent while the car is switched on
    {
      m_on_seen_secs = 5;
      break;
    }

    case 0x281: // Body Control Module: lock state
    {
      uint8_t state = d[1];
      static uint8_t last_state = 0x00;

      // 0xA9 = locked, 0xA8 = unlocked
      if (state != last_state && (state == 0xA9 || state == 0xA8))
      {
        bool locked = (state == 0xA9);

        // Standard OVMS metric, used by apps / HA:
        StdMetrics.ms_v_env_locked->SetValue(locked);

        ESP_LOGI(TAG, "Lock state changed: %s (CAN 0x281 byte1=0x%02x)",
                 locked ? "LOCKED" : "UNLOCKED", state);

        last_state = state;
      }
      break;
    }

    case 0x748: // The car's own telematics polling the BMS
    {
      // The car polls its BMS a few times a second while driving and every
      // few seconds while charging (either kind), but not when it merely
      // wakes because a door was opened. The 30 second timeout rides out
      // the gaps in the charging cadence.
      m_carpoll_seen_secs = 30;
      break;
    }

    case 0x795: // HV system status broadcast
    {
      HandleCharger795(d, p_frame->FIR.B.DLC);
      break;
    }

    case 0x540: // Odometer (from your trace)
    {
      // Frame example seen:
      // 540 00 00 00 00 90 f0 02 00
      //
      // Treat bytes [4..6] as 24-bit little-endian, resolution 0.1 km:
      //   raw = d4 + d5*256 + d6*65536
      //   km  = raw / 10
      uint32_t raw =
        (uint32_t)d[4] |
        ((uint32_t)d[5] << 8) |
        ((uint32_t)d[6] << 16);

      float km = raw / 10.0f;

      if (km > 0 && km < 1000000.0f)
      {
        if (StdMetrics.ms_v_pos_odometer->AsFloat() != km)
        {
          StdMetrics.ms_v_pos_odometer->SetValue(km);
          ESP_LOGI(TAG, "Odometer: %.1f km (raw=0x%06x)", km, raw);
        }
      }
      else
      {
        ESP_LOGW(TAG, "Odometer raw=0x%06x (%.1f km) out of range, ignored",
                 raw, km);
      }
      break;
    }

    default:
      break;
  }
}

// ─────────────────────────────────────────────
//  OBDII Poll Reply Handler
// ─────────────────────────────────────────────
void OvmsVehicleMaxt90::IncomingPollReply(const OvmsPoller::poll_job_t& job,
                                          uint8_t* data, uint8_t length)
{
  // A reply can arrive split over several frames (VIN, the cell voltage
  // array). Collect the pieces and decode once the last frame is in.
  if (job.mlframe == 0) {
    m_rxbuf.clear();
    m_rxbuf.reserve(length + job.mlremain);
  }
  m_rxbuf.append(reinterpret_cast<char*>(data), length);
  if (job.mlremain)
    return;

  const uint8_t* d = reinterpret_cast<const uint8_t*>(m_rxbuf.data());
  size_t len = m_rxbuf.size();

  switch (job.pid)
  {
    case 0xF190: { // VIN (ASCII)
      if (len >= 1) {
        std::string vin(reinterpret_cast<const char*>(d), len);
        StdMetrics.ms_v_vin->SetValue(vin);
        ESP_LOGD(TAG, "VIN: %s", vin.c_str());
      }
      break;
    }

    case 0xE002: { // SOC (%)
      if (len >= 1) {
        float soc = d[0];
        if (soc > 0 && soc <= 100) {
          if (StdMetrics.ms_v_bat_soc->AsFloat() != soc) {
            StdMetrics.ms_v_bat_soc->SetValue(soc);
            ESP_LOGD(TAG, "SOC: %.0f %%", soc);
          }
        } else {
          ESP_LOGW(TAG,
                   "Invalid SOC %.1f ignored (car likely off or poll timeout)",
                   soc);
        }
      }
      break;
    }

    case 0xE003: { // SOH (%)
      if (len >= 2) {
        uint16_t raw = u16be(d);
        float soh = raw / 100.0f;

        // Filter out bogus default values (0xFFFF, 0x1800 = 61.44%, etc.)
        if (raw == 0xFFFF || raw == 0x1800 || soh <= 50.0f || soh > 150.0f) {
          ESP_LOGW(TAG, "Invalid SOH raw=0x%04x (%.2f %%) ignored", raw, soh);
          break;
        }

        if (StdMetrics.ms_v_bat_soh->AsFloat() != soh) {
          StdMetrics.ms_v_bat_soh->SetValue(soh);
          ESP_LOGD(TAG, "SOH: %.2f %%", soh);
        }
      }
      break;
    }

    case 0xE004: { // Unknown drive value (NOT a READY flag)
      // This was first read as a READY bitfield, but drive captures show it
      // varying with load on the move and sitting at zero while coasting or
      // charging. It looks like a power or torque value with an unknown
      // scale. The car being on is detected from the 0x266 broadcast
      // instead; this is only logged to help decode it later.
      if (len >= 2)
        ESP_LOGD(TAG, "E004 (unknown drive value): 0x%04x", u16be(d));
      break;
    }

    case 0xE009: { // Plug present (u16)
      // The low byte is 0x00 with a cable in (verified while charging AC
      // and DC, and parked with the cable in) and non-zero while driving.
      if (len >= 2) {
        uint16_t v = u16be(d);
        bool plug_present = ((v & 0x00FF) == 0x00);
        StdMetrics.ms_v_charge_pilot->SetValue(plug_present);
        StdMetrics.ms_v_door_chargeport->SetValue(plug_present);
        ESP_LOGD(TAG, "Plug present: raw=0x%04x plug=%s",
                 v, plug_present ? "true" : "false");
      }
      break;
    }

    case 0xE010: { // HVAC/Coolant temperature (°C)
      if (len >= 2 && m_hvac_temp_c) {
        uint16_t raw = u16be(d);
        float t = raw / 10.0f;

        bool env_on = StdMetrics.ms_v_env_on->AsBool();

        // Ignore the constant bogus 45.8 °C we see when the car is off
        if (!env_on && raw == 458) {
          ESP_LOGW(TAG,
                   "HVAC temp raw=0x%04x (%.1f °C) ignored (car off/default)",
                   raw, t);
          break;
        }

        // Filter out known bogus patterns (default buffer or timeout)
        if (raw == 0x0200 || raw == 0xFFFF || t < -40 || t > 125) {
          ESP_LOGW(TAG, "Invalid HVAC temp raw=0x%04x (%.1f °C) ignored",
                   raw, t);
          break;
        }

        m_hvac_temp_c->SetValue(t);
        ESP_LOGD(TAG, "HVAC/Coolant temp: %.1f °C", t);
      }
      break;
    }

    case 0xE025: { // Ambient temperature (°C)
      if (len >= 2) {
        uint16_t raw = u16be(d);
        float ta = raw / 10.0f;

        bool env_on = StdMetrics.ms_v_env_on->AsBool();

        // Ignore the constant bogus 7.5 °C we see when the car is off
        if (!env_on && raw == 75) {
          ESP_LOGW(TAG,
                   "Ambient temp raw=0x%04x (%.1f °C) ignored (car off/default)",
                   raw, ta);
          break;
        }

        // Filter out default/bogus data
        if (raw == 0x0200 || raw == 0xFFFF || ta < -50 || ta > 80) {
          ESP_LOGW(TAG, "Invalid ambient temp raw=0x%04x (%.1f °C) ignored",
                   raw, ta);
          break;
        }

        StdMetrics.ms_v_env_temp->SetValue(ta);
        ESP_LOGD(TAG, "Ambient temp: %.1f °C", ta);
      }
      break;
    }

    case 0xE001: { // AC charge current from the on-board charger (0x722)
      // Checked against a metered wall plug: reads 0 when idle, 7 at ~8 A
      // and 12 at ~13 A, so the value is whole amps only. The on-board
      // charger (0x7a2) should be the only module answering this PID, but
      // check the module id anyway.
      if (job.moduleid_rec == 0x7a2 && len >= 1) {
        float amps = d[0];
        StdMetrics.ms_v_charge_current->SetValue(amps);
        ESP_LOGD(TAG, "Charge current: %.0f A", amps);
      }
      break;
    }

    case 0xE022: { // 12V system current from the VCU
      // Read 0xA3 = 163 while charging, which matches a plausible 16.3 A,
      // so we take it as 0.1 A units (medium confidence).
      if (job.moduleid_rec == 0x7eb && len >= 1) {
        float amps = ((len >= 2) ? u16be(d) : d[0]) / 10.0f;
        StdMetrics.ms_v_bat_12v_current->SetValue(amps);
      }
      break;
    }

    case 0xB105: { // Pack voltage from the BMS (u16, 0.01 V units)
      if (job.moduleid_rec == 0x7c8 && len >= 2) {
        float volts = u16be(d) / 100.0f;
        if (volts > 100 && volts < 500)
          StdMetrics.ms_v_bat_voltage->SetValue(volts);
      }
      break;
    }

    case 0xB114: { // Cell voltage max / min / average (3 x u16, mV)
      if (job.moduleid_rec == 0x7c8 && len >= 6) {
        StdMetrics.ms_v_bat_pack_vmax->SetValue(u16be(d) / 1000.0f);
        StdMetrics.ms_v_bat_pack_vmin->SetValue(u16be(d + 2) / 1000.0f);
        StdMetrics.ms_v_bat_pack_vavg->SetValue(u16be(d + 4) / 1000.0f);
      }
      break;
    }

    case 0xB110: { // Pack temperature sensors (3 x s16, 0.1 °C units)
      if (job.moduleid_rec == 0x7c8 && len >= 6) {
        float sum = 0;
        BmsRestartCellTemperatures();
        for (int i = 0; i < 3; i++) {
          float t = (int16_t)u16be(d + i * 2) / 10.0f;
          BmsSetCellTemperature(i, t);
          sum += t;
        }
        StdMetrics.ms_v_bat_temp->SetValue(sum / 3.0f);
      }
      break;
    }

    case 0xB136: { // 12V DC-DC output voltage (u16, 0.01 V units)
      if (job.moduleid_rec == 0x7c8 && len >= 2 && m_dcdc_voltage)
        m_dcdc_voltage->SetValue(u16be(d) / 100.0f);
      break;
    }

    case 0xB142: { // Full cell voltage array (u16 mV per cell, multi-frame)
      // The BMS reports 110 values of about 3.31 V each. The pack maths
      // (88.5 kWh, 270 Ah, 327 V from 0xB105) points at roughly 99 series
      // cells, so the extra entries are unexplained. We show what the BMS
      // reports and size the monitor from the reply.
      if (job.moduleid_rec == 0x7c8 && len >= 4 && (len % 2) == 0) {
        int cells = len / 2;
        if (cells != m_bms_cells) {
          BmsSetCellArrangementVoltage(cells, 10);
          m_bms_cells = cells;
        }
        BmsRestartCellVoltages();
        for (int i = 0; i < cells; i++)
          BmsSetCellVoltage(i, u16be(d + i * 2) / 1000.0f);
      }
      break;
    }

    default:
      break;
  }
}

// ─────────────────────────────────────────────
//  HV System Status (0x795 broadcast)
// ─────────────────────────────────────────────
//
// The car broadcasts frame 0x795 whenever the HV system is up. It was first
// taken for a charger status frame, but drive captures show the exact same
// payload while driving, while AC charging and while DC charging:
//
//   HV live    : 00 0a a4 00 28 00 00 00
//   powering up: 00 00 00 00 00 00 00 00
//   HV down    : frame stops being sent entirely
//
// Bytes 1-2 (0x0AA4) and byte 4 (0x28) read the same at 6 to 13 A AC and at
// a 50 kW DC charge, so they are constants, not measurements. Bit 0x40 of
// byte 4 shows briefly while a charge session is being negotiated.
//
// So this frame only tells us the HV system is live. Whether that means
// driving or charging is worked out in Ticker1.
void OvmsVehicleMaxt90::HandleCharger795(const uint8_t* d, uint8_t length)
{
  for (int i = 0; i < length; i++) {
    if (d[i] != 0) {
      m_hv_seen_secs = 10;
      return;
    }
  }
}

// Charge detection settled (see Ticker1 for the conditions):
void OvmsVehicleMaxt90::SetChargeStarted()
{
  ESP_LOGI(TAG, "Charge started (HV live, car off, plug in, car polling)");

  m_charge_secs = 0;
  StdMetrics.ms_v_charge_inprogress->SetValue(true);
  StdMetrics.ms_v_charge_mode->SetValue("standard");
  StdMetrics.ms_v_charge_state->SetValue("charging");
  StdMetrics.ms_v_charge_substate->SetValue("onrequest");
  // The charge type is settled a few seconds in: "type2" once the on-board
  // charger reports AC current, "ccs" if it stays at zero (see Ticker1).
  StdMetrics.ms_v_charge_type->SetValue("");
}

// Charging has ended, either because the HV system shut down or because the
// car was switched on to drive away. The plug and charge port metrics are
// left to the E009 poll, which owns them.
void OvmsVehicleMaxt90::SetChargeStopped()
{
  ESP_LOGI(TAG, "Charge stopped");

  StdMetrics.ms_v_charge_inprogress->SetValue(false);
  StdMetrics.ms_v_charge_current->SetValue(0);
  StdMetrics.ms_v_charge_state->SetValue(
    (StdMetrics.ms_v_bat_soc->AsFloat() >= 99) ? "done" : "stopped");
  StdMetrics.ms_v_charge_substate->SetValue("");
}

// The framework calls this once a second, just before the next poll is
// sent. It's the only place the poll state changes, so the charge and
// ready signals can't fight over it:
//   charging -> 2,  ready (on) -> 1,  otherwise -> 0
void OvmsVehicleMaxt90::PollerStateTicker(canbus* bus)
{
  bool charging = StdMetrics.ms_v_charge_inprogress->AsBool();

  uint8_t want;
  if (charging)
    want = 2;
  else if (StdMetrics.ms_v_env_on->AsBool())
    want = 1;
  else
    want = 0;

  if (m_poll_state != want) {
    ESP_LOGI(TAG, "Poll state %d -> %d (charge=%s on=%s)",
             m_poll_state, want,
             charging ? "true" : "false",
             StdMetrics.ms_v_env_on->AsBool() ? "true" : "false");
    PollSetState(want);
  }
}

// ─────────────────────────────────────────────
//  1 Hz Tick: on / charging detection
// ─────────────────────────────────────────────
void OvmsVehicleMaxt90::Ticker1(uint32_t ticker)
{
  if (m_hv_seen_secs > 0)      m_hv_seen_secs--;
  if (m_on_seen_secs > 0)      m_on_seen_secs--;
  if (m_carpoll_seen_secs > 0) m_carpoll_seen_secs--;

  bool hv_live = (m_hv_seen_secs > 0);
  bool on      = (m_on_seen_secs > 0);

  // The car is on while the 0x266 powertrain broadcast keeps arriving:
  if (StdMetrics.ms_v_env_on->AsBool() != on) {
    ESP_LOGI(TAG, "Vehicle switched %s (0x266 broadcast %s)",
             on ? "on" : "off", on ? "present" : "gone");
    StdMetrics.ms_v_env_on->SetValue(on);
  }

  if (StdMetrics.ms_v_charge_inprogress->AsBool()) {
    m_charge_pending_secs = 0;
    m_charge_secs++;

    // The charge is over when the HV system shuts down, or when the driver
    // switches the car on to leave:
    if (!hv_live || on) {
      SetChargeStopped();
    }
    else if (StdMetrics.ms_v_charge_current->AsFloat() > 0) {
      // The on-board charger reports AC amps, so this is an AC charge:
      if (StdMetrics.ms_v_charge_type->AsString() != "type2")
        StdMetrics.ms_v_charge_type->SetValue("type2");
    }
    else if (m_charge_secs == 20) {
      // 20 seconds in and the on-board charger still reports no AC
      // current, so this is a DC charge:
      StdMetrics.ms_v_charge_type->SetValue("ccs");
    }
  }
  else {
    // Charging looks like: HV system live, car not switched on, cable in,
    // and the car's telematics polling its own ECUs. Each condition rules
    // out a false positive seen in real captures. Driving is ruled out by
    // the on check. After switching off, the HV system and the polling run
    // on for half a minute, but the cable is out. Opening a door with the
    // cable in wakes the HV system, but the car doesn't poll.
    if (hv_live && !on &&
        StdMetrics.ms_v_charge_pilot->AsBool() && m_carpoll_seen_secs > 0) {
      if (++m_charge_pending_secs >= 10)
        SetChargeStarted();
    } else {
      m_charge_pending_secs = 0;
    }
  }
}

// ─────────────────────────────────────────────
//   Module Registration
// ─────────────────────────────────────────────

class OvmsVehicleMaxt90Init
{
public:
  OvmsVehicleMaxt90Init();
} MyOvmsVehicleMaxt90Init __attribute__((init_priority(9000)));

OvmsVehicleMaxt90Init::OvmsVehicleMaxt90Init()
{
  ESP_LOGI(TAG, "Registering Vehicle: Maxus T90 EV (9000)");
  // Vehicle type string "MT90" is the type code in OVMS:
  MyVehicleFactory.RegisterVehicle<OvmsVehicleMaxt90>(
    "MT90", "Maxus T90 EV");
}

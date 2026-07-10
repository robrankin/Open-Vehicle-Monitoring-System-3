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

  // Define poll list:
  //  - State 0: vehicle off
  //  - State 1: vehicle on / driving
  //  - State 2: charging (detected from the 0x795 charger broadcast)
  //
  // Only READY (0xE004) is polled in state 0, so we don't keep the ECUs awake.
  // The state-2 columns keep SOC, SOH and the temperatures updating while
  // the car charges.
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

    // READY flag, polled in all states, faster in "off" to detect wake
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE004,
      { 5, 10, 10 }, 0, ISOTP_STD },

    // Plug present
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE009,
      { 0, 10, 10 }, 0, ISOTP_STD },

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

    case 0x795: // Charger status broadcast (only sent while an EVSE is connected)
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
  switch (job.pid)
  {
    case 0xF190: { // VIN (ASCII)
      if (length >= 1) {
        std::string vin(reinterpret_cast<char*>(data),
                        reinterpret_cast<char*>(data) + length);
        StdMetrics.ms_v_vin->SetValue(vin);
        ESP_LOGD(TAG, "VIN: %s", vin.c_str());
      }
      break;
    }

    case 0xE002: { // SOC (%)
      if (length >= 1) {
        float soc = data[0];
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
      if (length >= 2) {
        uint16_t raw = u16be(data);
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

    case 0xE004: { // READY bitfield
      if (length >= 2) {
        uint16_t v = u16be(data);
        bool ready = (v & 0x000C) != 0;
        bool prev_ready = StdMetrics.ms_v_env_on->AsBool();
        StdMetrics.ms_v_env_on->SetValue(ready);

        if (ready != prev_ready) {
          ESP_LOGI(TAG, "READY flag changed: raw=0x%04x ready=%s",
                   v, ready ? "true" : "false");
          // We don't change the poll state here. PollerStateTicker() works it
          // out from the charge and ready flags together, so a "not ready"
          // reading while charging can't knock us out of state 2.
        }
      }
      break;
    }

    case 0xE009: { // Plug present (u16)
      if (length >= 2) {
        uint16_t v = u16be(data);
        bool plug_present = ((v & 0x00FF) == 0x00);
        StdMetrics.ms_v_charge_pilot->SetValue(plug_present);
        ESP_LOGD(TAG, "Plug present: raw=0x%04x plug=%s",
                 v, plug_present ? "true" : "false");
      }
      break;
    }

    case 0xE010: { // HVAC/Coolant temperature (°C)
      if (length >= 2 && m_hvac_temp_c) {
        uint16_t raw = u16be(data);
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
      if (length >= 2) {
        uint16_t raw = u16be(data);
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
      if (job.moduleid_rec == 0x7a2 && length >= 1) {
        float amps = data[0];
        StdMetrics.ms_v_charge_current->SetValue(amps);
        ESP_LOGD(TAG, "Charge current: %.0f A", amps);
      }
      break;
    }

    default:
      break;
  }
}

// ─────────────────────────────────────────────
//  Charge Status (0x795 broadcast)
// ─────────────────────────────────────────────
//
// The car broadcasts charger status frame 0x795 about 7 times a second,
// but only while an EVSE is physically connected. Payloads observed on an
// AC granny lead:
//
//   charging : 00 0a a4 00 28 00 00 00
//   plugged  : 00 00 00 00 00 00 00 00   (connected but not delivering)
//   unplugged: frame stops being sent entirely
//
// So receiving the frame at all means the pilot is present, and a non-zero
// payload means the car is charging.
//
// Byte 4 (0x28) is not the charge current. It reads the same 0x28 at 6 A,
// 8 A and 13 A charge rates (checked against a metered smart plug), so it
// looks like a constant or a mode flag. The real charge current comes from
// the on-board charger via the 0xE001 poll.
void OvmsVehicleMaxt90::HandleCharger795(const uint8_t* d, uint8_t length)
{
  // Any 0x795 frame means an EVSE is connected:
  m_evse_seen_secs = 10;
  StdMetrics.ms_v_charge_pilot->SetValue(true);
  StdMetrics.ms_v_door_chargeport->SetValue(true);

  // A non-zero payload means current is flowing. A zero payload means the
  // car is plugged in but not charging; we don't declare the charge stopped
  // here, the m_charge_seen_secs timeout in Ticker1 does that.
  bool current_flowing = (length >= 5) && (d[4] != 0);

  if (current_flowing) {
    m_charge_seen_secs = 5; // ride out brief drop-outs before calling it stopped

    if (!StdMetrics.ms_v_charge_inprogress->AsBool()) {
      ESP_LOGI(TAG, "Charge started (0x795 payload non-zero)");

      StdMetrics.ms_v_charge_inprogress->SetValue(true);
      StdMetrics.ms_v_charge_mode->SetValue("standard");
      StdMetrics.ms_v_charge_state->SetValue("charging");
      StdMetrics.ms_v_charge_substate->SetValue("onrequest");
      // We've only seen AC (type2) charging so far. DC/CCS detection is
      // still to do.
      StdMetrics.ms_v_charge_type->SetValue("type2");
    }
  }
}

// Charging has ended. If evse_present is false the cable is unplugged too.
void OvmsVehicleMaxt90::SetChargeStopped(bool evse_present)
{
  if (StdMetrics.ms_v_charge_inprogress->AsBool())
    ESP_LOGI(TAG, "Charge stopped (evse_present=%s)",
             evse_present ? "true" : "false");

  StdMetrics.ms_v_charge_inprogress->SetValue(false);
  StdMetrics.ms_v_charge_current->SetValue(0);
  StdMetrics.ms_v_charge_state->SetValue(evse_present ? "stopped" : "");
  StdMetrics.ms_v_charge_substate->SetValue("");

  StdMetrics.ms_v_charge_pilot->SetValue(evse_present);
  StdMetrics.ms_v_door_chargeport->SetValue(evse_present);
  if (!evse_present) {
    StdMetrics.ms_v_charge_mode->SetValue("");
    StdMetrics.ms_v_charge_type->SetValue("");
  }
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
//  1 Hz Tick (0x795 broadcast timeouts)
// ─────────────────────────────────────────────
void OvmsVehicleMaxt90::Ticker1(uint32_t ticker)
{
  // If the current stops while the cable is still plugged in, the charge
  // has stopped.
  if (m_charge_seen_secs > 0) {
    if (--m_charge_seen_secs == 0 &&
        StdMetrics.ms_v_charge_inprogress->AsBool() && m_evse_seen_secs > 0)
      SetChargeStopped(/*evse_present=*/true);
  }

  // When 0x795 stops arriving entirely, the cable has been unplugged.
  if (m_evse_seen_secs > 0) {
    if (--m_evse_seen_secs == 0)
      SetChargeStopped(/*evse_present=*/false);
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

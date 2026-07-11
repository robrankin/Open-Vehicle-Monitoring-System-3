#ifndef __VEHICLE_MAXT90_H__
#define __VEHICLE_MAXT90_H__

#include <string>

#include "vehicle_obdii.h"
#include "ovms_metrics.h"

class OvmsVehicleMaxt90 : public OvmsVehicleOBDII
{
public:
  OvmsVehicleMaxt90();
  ~OvmsVehicleMaxt90();

protected:
  // OBDII poll replies:
  void IncomingPollReply(const OvmsPoller::poll_job_t& job,
                         uint8_t* data, uint8_t length) override;

  // Raw CAN1 frames (lock status on 0x281, odometer on 0x540,
  // HV system status on 0x795, etc.):
  void IncomingFrameCan1(CAN_frame_t* p_frame) override;

  // 1 Hz tick: counts down the broadcast timers and works out the
  // on / charging state from them:
  void Ticker1(uint32_t ticker) override;

  // Works out the poll state (charging -> 2, on -> 1, off -> 0). Called by
  // the framework once a second, just before the next poll goes out:
  void PollerStateTicker(canbus* bus) override;

private:
  // Custom metrics:
  //  - xmt.v.hvac.temp    : HVAC / coolant temperature (°C)
  //  - xmt.b.capacity     : Nominal pack capacity (kWh)
  //  - xmt.v.dcdc.voltage : 12V DC-DC output voltage (V)
  OvmsMetricFloat* m_hvac_temp_c       = nullptr; // xmt.v.hvac.temp
  OvmsMetricFloat* m_pack_capacity_kwh = nullptr; // xmt.b.capacity
  OvmsMetricFloat* m_dcdc_voltage      = nullptr; // xmt.v.dcdc.voltage

  // Reassembly buffer for poll replies that span several frames
  // (VIN, BMS cell voltage array):
  std::string m_rxbuf;

  // Cell count last reported by BMS DID 0xB142; sets the BMS monitor
  // arrangement when it changes:
  int m_bms_cells = 110;

  // On / charging detection state. The three countdown timers are refreshed
  // by traffic seen in IncomingFrameCan1 and counted down once a second by
  // Ticker1, so each condition clears itself when its frames stop:
  //  - m_hv_seen_secs:      0x795 with a non-zero payload. The HV system is
  //                         live, which happens when driving and during both
  //                         AC and DC charging.
  //  - m_on_seen_secs:      0x266 powertrain broadcast, only sent while the
  //                         car is switched on. Drives ms_v_env_on.
  //  - m_carpoll_seen_secs: the car's own telematics polling the BMS (0x748).
  //                         Seen while driving or charging, but not when the
  //                         car merely wakes because a door was opened.
  int m_hv_seen_secs      = 0;
  int m_on_seen_secs      = 0;
  int m_carpoll_seen_secs = 0;

  // Seconds the charge conditions have held (HV live, car off, plug in,
  // car polling). The charge is declared once this reaches 10:
  int m_charge_pending_secs = 0;

  // Seconds since the charge started. Used to settle on "ccs" as the charge
  // type when the on-board charger never reports any AC current:
  int m_charge_secs = 0;

  // Charge status handlers / helpers:
  void HandleCharger795(const uint8_t* d, uint8_t length);
  void SetChargeStarted();
  void SetChargeStopped();

  // Helpers:
  static inline uint16_t u16be(const uint8_t* p)
  {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
  }
};

#endif // __VEHICLE_MAXT90_H__

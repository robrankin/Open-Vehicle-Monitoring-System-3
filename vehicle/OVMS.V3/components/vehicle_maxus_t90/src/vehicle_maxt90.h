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

  // OBDII poll transmit result. The framework calls this on every poll TX,
  // with success == false when the frame was not acknowledged on the bus.
  // We use it to notice the VCU going unreachable while parked (see the
  // parked-backoff note below):
  void IncomingPollTxCallback(const OvmsPoller::poll_job_t& job,
                              bool success) override;

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
  //  - xmt.v.hvac.temp     : HVAC / coolant temperature (°C)
  //  - xmt.b.capacity      : Nominal pack capacity (kWh)
  //  - xmt.v.dcdc.voltage  : 12V DC-DC output voltage (V)
  //  - xmt.b.voltage.limit : BMS computed voltage limit (V), see 0xB105
  OvmsMetricFloat* m_hvac_temp_c        = nullptr; // xmt.v.hvac.temp
  OvmsMetricFloat* m_pack_capacity_kwh  = nullptr; // xmt.b.capacity
  OvmsMetricFloat* m_dcdc_voltage       = nullptr; // xmt.v.dcdc.voltage
  OvmsMetricFloat* m_batt_voltage_limit = nullptr; // xmt.b.voltage.limit

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

  // Parked no-poll backoff.
  //
  // The only poll that runs while parked (poll state 0) is the plug detect
  // to the VCU (0x7e3). While the car is off the VCU is asleep and never
  // acknowledges, so every one of those transmits is a CAN "no-ack" error
  // that pushes the transmit error counter up until the bus trips to
  // bus-off and has to be reset. Receiving is unaffected, so the passive
  // broadcasts we rely on for on/charge detection keep arriving.
  //
  // So we count the VCU's failed transmits (a bus-ack from another ECU does
  // not clear the count - only a real wake broadcast does): once enough have
  // failed the VCU is asleep and we move the poller to state 3, which polls
  // nothing (no poll list entry sets a state-3 interval, so they default to
  // 0). We leave that dormant state as soon as any live broadcast shows the
  // bus is awake again - which always happens before a charge or drive.
  uint8_t m_vcu_txfail_streak = 0;    // failed VCU transmits since last wake
  bool    m_dormant           = false;
  static const uint8_t kVcuAsleepThreshold = 5;  // failed sends before dormant
  static const uint8_t kDormantPollState   = 3;  // unused state = polls nothing

  // Leave the parked backoff (called when a live broadcast is seen):
  void ArmFromDormant();

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

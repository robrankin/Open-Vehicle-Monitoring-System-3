#ifndef __VEHICLE_MAXT90_H__
#define __VEHICLE_MAXT90_H__

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
  // charger status on 0x795, etc.):
  void IncomingFrameCan1(CAN_frame_t* p_frame) override;

  // 1 Hz tick (used to time out the charger-status broadcast):
  void Ticker1(uint32_t ticker) override;

  // Works out the poll state (charging -> 2, on -> 1, off -> 0). Called by
  // the framework once a second, just before the next poll goes out:
  void PollerStateTicker(canbus* bus) override;

private:
  // Custom metrics:
  //  - xmt.v.hvac.temp  : HVAC / coolant temperature (°C)
  //  - xmt.b.capacity   : Nominal pack capacity (kWh)
  OvmsMetricFloat* m_hvac_temp_c       = nullptr; // xmt.v.hvac.temp
  OvmsMetricFloat* m_pack_capacity_kwh = nullptr; // xmt.b.capacity

  // Charge state, driven by the 0x795 charger-status broadcast. Ticker1
  // counts these down once a second and the broadcast resets them, so when
  // the frames stop arriving the state clears itself. The charging flag
  // itself is the standard ms_v_charge_inprogress metric.
  int  m_evse_seen_secs  = 0;     // >0 while 0x795 is being received (plugged in)
  int  m_charge_seen_secs = 0;    // >0 while charge current is being seen

  // Charge status handlers / helpers:
  void HandleCharger795(const uint8_t* d, uint8_t length);
  void SetChargeStopped(bool evse_present);

  // Helpers:
  static inline uint16_t u16be(const uint8_t* p)
  {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
  }
};

#endif // __VEHICLE_MAXT90_H__

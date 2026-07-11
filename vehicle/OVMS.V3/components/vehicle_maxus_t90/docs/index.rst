Maxus T90 EV (MT90)
===================

Vehicle type code: ``MT90``

The Maxus T90 EV module provides basic battery, charging, temperature and
lock/odometer integration using the vehicle OBD-II port and a single CAN bus
at 500 kbps.

The module is still under active development; this page reflects the current
feature set.


Hardware & Installation
-----------------------

.. list-table::
   :widths: 40 60
   :header-rows: 1

   * - Item
     - Notes
   * - OVMS hardware
     - OVMS v3 module (or later)
   * - Vehicle connection
     - OBD-II port using the standard OVMS OBD-II to DB9 data cable
   * - CAN bus
     - CAN1 at 500 kbps, active mode
   * - GPS / GSM antennas
     - Standard OVMS antennas (or compatible) as per OVMS documentation


Feature Coverage
----------------

.. list-table::
   :widths: 45 15 40
   :header-rows: 1

   * - Function
     - Status
     - Notes
   * - SOC display
     - Yes
     - From OBD-II PID ``0xE002`` → ``ms_v_bat_soc``
   * - SOH display
     - Yes
     - From OBD-II PID ``0xE003`` → ``ms_v_bat_soh`` (with filtering)
   * - Battery capacity
     - Yes
     - Custom metric ``xmt.b.capacity`` (fixed 88.5 kWh)
   * - Pack voltage
     - Yes
     - From the BMS cell data → ``ms_v_bat_voltage``: average cell voltage
       (``0xB114``, every 30 s) times the cell count, refined by the exact
       cell sum (``0xB142``, every 60 s). The pack is 110 LFP cells in
       series, about 352 V nominal. (``0xB105`` looked like pack voltage
       at first but is a computed limit; see the implementation notes.)
   * - Cell voltages (BMS monitor)
     - Yes
     - From BMS DID ``0xB142`` (full array, polled every 60 s) plus
       ``0xB114`` (max / min / average). The BMS reports 110 cells and all
       110 values move independently across charge and drive captures, so
       the count is real.
   * - BMS voltage limit
     - Yes
     - Custom metric ``xmt.b.voltage.limit`` from BMS DID ``0xB105``.
       Behaves like a computed lower voltage limit, not a measurement.
   * - Battery temperatures
     - Yes
     - From BMS DID ``0xB110`` (3 sensors) → BMS monitor and
       ``ms_v_bat_temp``
   * - 12V battery current
     - Yes
     - From VCU PID ``0xE022`` → ``ms_v_bat_12v_current`` (0.1 A units,
       medium confidence)
   * - 12V DC-DC output voltage
     - Yes
     - Custom metric ``xmt.v.dcdc.voltage`` (BMS ``0xB136``)
   * - Odometer
     - Yes
     - From CAN ID ``0x540`` → ``ms_v_pos_odometer`` (0.1 km resolution)
   * - Vehicle READY / ignition state
     - Yes
     - From the ``0x266`` powertrain broadcast, which the car only sends
       while switched on → ``ms_v_env_on`` and poll state control
   * - Lock status
     - Yes
     - From CAN ID ``0x281`` → ``ms_v_env_locked`` (locked/unlocked)
   * - Charge plug / pilot present
     - Yes
     - From OBD-II PID ``0xE009`` → ``ms_v_charge_pilot``
   * - Cabin / coolant temperature
     - Yes
     - From OBD-II PID ``0xE010`` → custom metric ``xmt.v.hvac.temp``
   * - Ambient temperature
     - Yes
     - From OBD-II PID ``0xE025`` → ``ms_v_env_temp``
   * - GPS location
     - Yes
     - Provided by the OVMS modem GPS (not vehicle-specific)
   * - Speed display
     - No (vehicle-specific)
     - Not present on the powertrain CAN (checked against drive captures);
       only GPS-based speed available via OVMS core
   * - Charge state / in progress
     - Yes
     - Worked out from several signals together (see the implementation
       notes) → ``ms_v_charge_inprogress``, ``ms_v_charge_state``.
       Detects both AC and DC charging; verified against captures of a
       granny-lead AC charge and a 50 kW CCS rapid charge. While charging,
       the module also keeps SOC, SOH and temperatures updating.
   * - Charge current (AC)
     - Yes
     - From OBD-II PID ``0xE001`` on the on-board charger (``0x722``) →
       ``ms_v_charge_current``. Whole amps, within about 1 A of the real
       value. Checked against a metered plug at 6, 8, 10 and 13 A. Reads
       zero during a DC charge, where the on-board charger isn't involved.
   * - Charge power / energy counters
     - No
     - The charger ECU only gives whole-amp current and AC voltage, so
       there is nothing to calculate power or energy from
   * - Charge control (start/stop, limits)
     - No
     - Not yet implemented
   * - Charging interruption alerts
     - No
     - Not yet implemented
   * - Trip counters / consumption
     - No
     - Not yet implemented for MT90
   * - TPMS
     - No
     - No TPMS integration yet
   * - Door/window state
     - No
     - Only global lock/unlock is currently decoded
   * - Remote lock/unlock control
     - No
     - Read-only lock status only
   * - Pre-heat / HVAC remote control
     - No
     - Not yet implemented
   * - Valet mode
     - No
     - Not implemented for this vehicle
   * - AC / DC charge type detection
     - Yes
     - ``ms_v_charge_type`` reads ``type2`` once the on-board charger
       reports AC current, or ``ccs`` when it stays at zero during a charge


Implementation Notes
--------------------

* The module derives from ``OvmsVehicleOBDII`` and registers CAN1 at
  500 kbps in active mode.
* Polling is done on ECU ``0x7E3 / 0x7EB`` using extended OBD-II PIDs.
* The poller currently defines three poll states:
  
  * State 0: vehicle off  
  * State 1: vehicle on / driving  
  * State 2: charging

* Only the plug detect (PID ``0xE009``) is polled in state 0, so the ECUs
  aren't kept awake while parked. The other PIDs are only polled while the
  vehicle is on or charging.
* The car being on is detected from the ``0x266`` powertrain broadcast,
  which is only sent while the car is switched on. (PID ``0xE004`` was
  first read as a READY flag, but it varies with load while driving, so it
  is some other value and is no longer used for this.) The poll state is
  worked out once a second in ``PollerStateTicker()``: charging → 2,
  on → 1, otherwise → 0.
* Charge detection combines four signals, each of which rules out a false
  positive seen in real captures. Charging means all of: the ``0x795``
  broadcast reports the HV system live (it sends the same payload when
  driving and during both charge types, so it can't be used alone), the
  car is not switched on (no ``0x266``), the cable is in (``0xE009``),
  and the car's own telematics is polling the BMS (``0x748`` requests
  seen on the bus, which don't happen when the car just wakes because a
  door was opened). Countdown timers in ``Ticker1`` clear each condition
  when its frames stop.
* The charge type is settled a few seconds into the charge: ``type2``
  once the on-board charger reports AC current, ``ccs`` if it stays at
  zero.
* Battery data comes from the BMS on ``0x748/0x7C8``, which uses a
  ``0xB1xx`` DID map (not the ``0xE0xx`` map the eDeliver3 uses):
  ``B114`` cell max/min/average and ``B142`` full cell array (these feed
  the pack voltage), ``B110`` temperature sensors, ``B136`` DC-DC output
  voltage. These are only polled while the vehicle is on or charging, so
  the BMS is never queried while the car sleeps.
* ``B105`` looked like pack voltage at first (it reads a plausible 327 V),
  but it reads BELOW the pack's resting voltage while a DC rapid charge
  pushes 150 A in, which a terminal measurement can't do. It behaves like
  a computed lower voltage limit, so it is kept as the custom metric
  ``xmt.b.voltage.limit`` until its meaning is settled. It returns
  ``0x8000`` as a "no value yet" marker while the pack electronics start
  up; the module discards that. (``B12A`` looked like raw SOC in early
  scans but stays near 50% regardless of the actual state of charge, so
  it is not used.)
* Odometer is taken from CAN ID ``0x540`` using bytes [4..6] as a 24-bit
  little-endian value with 0.1 km resolution.
* Lock status is decoded from CAN ID ``0x281`` (body control module) using
  byte 1 values:

  * ``0xA9`` → locked  
  * ``0xA8`` → unlocked  

  and mapped to the standard metric ``ms_v_env_locked``.

* Multiple sanity filters are applied to SOH and temperature PIDs to discard
  default/bogus values that occur when the vehicle is off or the ECU returns
  fallback frames.


Planned / Potential Extensions
------------------------------

The following features are candidates for future updates once the relevant
PIDs and CAN messages have been fully reverse engineered:

* Charge power and energy counters, if a usable source turns up (the
  charger ECU has none).
* Pack DC current (not found in the BMS ``0xB1xx`` block or in any
  broadcast frame during a 150 A DC charge; finding it would unlock DC
  power and consumption figures).
* Speed (not present on the powertrain CAN; the standard OBD speed PID
  answers but always reads zero, so it likely needs the chassis CAN on
  OBD pins 3 and 11).
* Additional body / door / window state.
* Remote climate control and other remote vehicle actions, if feasible.

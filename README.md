# single_phase_inverter

single_phase_inverter provides a C++ control block for single-phase inverters,
including SOGI-based quadrature generation, a PLL, and dq current/voltage loops.

## Main features
- SOGI (Second-Order Generalized Integrator) to generate alpha/beta components.
- PLL and dq transforms to estimate grid angle and frequency.
- Forming and following modes with automatic sync logic.
- Current and voltage PI loops and duty-cycle computation.
- Accessors for measured dq quantities and power.

## Public API (summary)
- `singlePhaseInverter()`
- `int8_t init(inverter_mode mode, float32_t V_bus, float32_t grid_Vpk, float32_t grid_w0, float32_t Ts)`
- `void reset()`
- `float32_t calculateDuty(float32_t vgrid_meas, float32_t igrid_meas)`
- `dqo_t getVdq()`
- `dqo_t getVdqOut()`
- `clarke_t getIab()`
- `clarke_t getVab()`
- `clarke_t getVabOutput()`
- `dqo_t getIdq()`
- `dqo_t getIdqRefDelta()`
- `dqo_t getPower()`
- `float32_t getw()`
- `float32_t getTheta()`
- `bool getSync()`
- `void setVBus(float32_t V_bus)`
- `void setVdqRef(dqo_t Vdq_ref)`
- `void setIdqRef(dqo_t Idq_ref)`
- `void setPowerOn(bool power_on)`
- `void setSyncOff()`
- `void setMode(inverter_mode mode)`
- `void setWRef(float32_t w_ref)`

## Notes
- Depends on `transform`, `pid`, `trigo`, and math/type headers from the OwnTech stack.
- Add `src/singlePhaseInverter.cpp` and `src/sogi.cpp` to your build and include
  `singlePhaseInverter.h` from your application.

/*
 *
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file is a power control class for single phase inverters
 *
 * @author Regis Ruelland <rruelland@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

#ifndef SINGLEPHASEINVERTER_H
#define SINGLEPHASEINVERTER_H

#include "sogi.h"
#include "transform.h"
#include "filters.h"


typedef enum {
    FORMING = 0,
    FOLLOWING = 1
} inverter_mode;

/**
 * Modulation of the H-bridge, i.e. how getDutyLeg2() is computed from the
 * same output voltage as calculateDuty() (leg 1). Both legs share the same
 * carrier (no phase shift); the average output is (2 d1 - 1) V_bus in both.
 *
 * - UNIPOLAR: d2 = 1 - d1, leg 2 in buck convention (duty cycle applied to
 *   its high-side switch). Three-level output, ripple at 2 f_sw.
 * - BIPOLAR: d2 = d1, leg 2 in complementary (boost) convention (duty cycle
 *   applied to its low-side switch): leg 2's high side is the complement
 *   of leg 1's. Two-level output, ripple at f_sw, constant common mode.
 */
typedef enum {
    BIPOLAR = 0,
    UNIPOLAR = 1
} modulation_scheme;


class singlePhaseInverter {
public:
    // Constructor
    singlePhaseInverter();

    // Initialization function
    int8_t init(inverter_mode mode, modulation_scheme modulation, float32_t V_bus, float32_t grid_Vpk, float32_t grid_w0, float32_t Ts);

    // Reset function
    void reset();

    // Calculate function
    float32_t calculateDuty(float32_t vgrid_meas, float32_t igrid_meas);

    dqo_t getVdq();

    dqo_t getVdqOut();

    clarke_t getIab();

    clarke_t getVab();

    clarke_t getVabOutput();

    dqo_t getIdq();

    dqo_t getIdqRefDelta();

    dqo_t getPower();

    float32_t getw();

    float32_t getTheta();

    bool getSync();

    void setVBus(float32_t V_bus);

    void setVdqRef(dqo_t Vdq_ref);

    void setIdqRef(dqo_t Idq_ref);

    void setPowerOn(bool power_on);

    void setSyncOff();

    // Change operating mode at runtime, keeping stored parameters
    void setMode(inverter_mode mode);

    // Set angular frequency reference (rad/s)
    void setWRef(float32_t w_ref);

    // Current loop PI (d and q, both modes): proportional gain (V/A),
    // integral time (s) and output bound (+/- V, a correction added to
    // the voltage reference or the measured voltage). Replaces the
    // defaults, also after setMode(). Resets the current loops: call it
    // with the power off.
    void setCurrentPi(float32_t Kp, float32_t Ti, float32_t bound);

    // Voltage loop PI (d and q, FORMING): proportional gain (A/V),
    // integral time (s) and output bound (+/- A, added to the current
    // reference). Same rules as setCurrentPi().
    void setVoltagePi(float32_t Kp, float32_t Ti, float32_t bound);

    // Duty cycle for leg 2, see modulation_scheme for the leg 2 convention
    float32_t getDutyLeg2();

    // Modulation given to init()
    modulation_scheme getModulation();

    // Shift the leg 2 reference angle by +1 or -1 degree (tests)
    void stepPhaseShiftUp();
    void stepPhaseShiftDown();

    // Current leg 2 reference angle shift in degrees
    float32_t getPhaseShiftDeg();

private:
    // Internal state variables
    inverter_mode _mode;
    modulation_scheme _modulation;

    dqo_t _power;

    dqo_t _Vdq;
    dqo_t _Vdq_ref;
    dqo_t _Vdq_output;

    dqo_t _Idq;
    dqo_t _Idq_ref;
    dqo_t _Idq_ref_delta;

    clarke_t _Vab;
    clarke_t _Vab_output;
    clarke_t _Iab;
    float32_t _Vond;

    float32_t _grid_Vpk;
    float32_t _grid_w0;
    float32_t _Ts;

    Sogi _sogi_i;
    Sogi _sogi_v;

    PidParams _pll_pi_params;    ///< PI controller parameters.
    Pid _pll_pi;                 ///< PI controller.

    PidParams _current_pi_params;    ///< PI controller parameters for the current loop.
    Pid _current_d_pi;                 ///< PI controller for the current loop.
    Pid _current_q_pi;                 ///< PI controller for the current loop.

    PidParams _voltage_pi_params;    ///< PI controller parameters for the voltage loop.
    Pid _voltage_d_pi;                 ///< PI controller for the voltage loop.
    Pid _voltage_q_pi;                 ///< PI controller for the voltage loop.

    // PI parameters given by setCurrentPi() / setVoltagePi(), kept by init()
    bool _current_pi_custom = false;
    float32_t _current_kp = 0.0F;
    float32_t _current_ti = 0.0F;
    float32_t _current_bound = 0.0F;
    bool _voltage_pi_custom = false;
    float32_t _voltage_kp = 0.0F;
    float32_t _voltage_ti = 0.0F;
    float32_t _voltage_bound = 0.0F;


    float32_t _theta;        ///< Current phase angle.
    float32_t _w;            ///< Angular frequency.
    float32_t _w_ref;        ///< Reference angular frequency.

    float32_t _duty_cycle;            ///< Internal duty cycle.
    float32_t _phase_shift_leg2_rad;  ///< Extra phase offset applied to leg 2 (rad).

    float32_t _V_bus;   ///< Internal bus voltage.

    bool _power_on;
    bool _sync;
    const float32_t _sync_power_tolerance = 0.1;

    uint16_t _sync_delay_counter; 
    uint16_t _sync_min_delay;



};

#endif // POWER_AC1PHASE_H

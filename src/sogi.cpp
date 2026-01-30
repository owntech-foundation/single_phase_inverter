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
 * @brief  This file contains the implementation of the SOGI-PLL class used in single phase inverters.
 *
 * @author Regis Ruelland <rruelland@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

#include "trigo.h"
#include "sogi.h"

/**
 * @brief Constructor for the SogiPLL class.
 */
Sogi::Sogi() {
    // Initialize members if needed
}

/**
 * @brief Initializes the SOGI-PLL system.
 *
 * @param Kr Resonance gain.
 * @param Ts Sampling time.
 */
void Sogi::init(float32_t Kr, float32_t Ts) {

    _Kr = Kr;
    _Ts = Ts;
    _den[0] = 0.0;
    _den[1] = 0.0;
    _den[2] = 0.0;
    _num[0] = 0.0;
    _num[1] = 0.0;
}

/**
 * @brief Performs SOGI calculation.
 *
 * @param input Input signal.
 * @param w0 Angular frequency.
 * @param params Reference to the SOGI parameters structure.
 * @return Clarke transformation result of the input signal.
 */
clarke_t Sogi::calc(float32_t input, float32_t w0) {

    float32_t coswt = 1.0F - 0.5F * (w0 * _Ts) * (w0 * _Ts);
    float32_t inv_sinwt = 1.0F / (w0 * _Ts);
    clarke_t result;

    _num[0] = _Kr * (input - _den[0]);
    _den[0] = _Ts * (_num[0] - coswt * _num[1]) + 2.0F * coswt * _den[1] - _den[2];
    result.alpha = _den[0];
    result.beta = -coswt * inv_sinwt * _den[0] + inv_sinwt * _den[1];
    result.o = 0.0;
    _num[1] = _num[0];
    _den[2] = _den[1];
    _den[1] = _den[0];

    return result;
}

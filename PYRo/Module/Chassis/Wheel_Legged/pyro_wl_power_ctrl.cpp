/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-05-07
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-07 17:47:00
 * @Description:
 *   Per-motor power control for wheel-legged chassis.
 *
 *   ── Torque decomposition ─────────────────────────────────────────────────
 *   tau_total_i  =  tau_motion_i  +  tau_balance_i
 *
 *   tau_balance_i  (Uelse = Upitch + Uleg)  — NEVER modified.
 *   tau_motion_i   (Uspeed ± Uyaw, treated as a whole) — restricted per motor.
 *
 *   ── Per-motor independent restriction ───────────────────────────────────
 *   Power model for motor i:
 *     P_i = tau_i * omega_i  +  k1[i] * |omega_i|  +  k2[i] * tau_i²  +  k3[i]
 *
 *   When total commanded power exceeds P_max, the budget is distributed
 *   proportionally:
 *     P_alloc_i = P_max * P_cmd_i / sum_j(P_cmd_j)
 *
 *   For each motor, restricted total torque is found by solving:
 *     k2[i] * x²  +  omega_i * x  +  ( k1[i]*|omega_i| + k3[i] - P_alloc_i )  =  0
 *   selecting the root sign-matched to tau_total_cmd_i.
 *
 *   Balance protection: if P_alloc_i ≤ P(tau_balance_i, omega_i), motion
 *   torque is zeroed and tau_balance_i is passed through unchanged.
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#include "pyro_wl_power_ctrl.h"

#include <cmath>

namespace pyro
{

/* ─────────────────────────────────────────────────────────────────────────── */
/* Public interface                                                             */
/* ─────────────────────────────────────────────────────────────────────────── */

status_t wl_power_ctrl_t::init(const wl_power_ctrl_cfg_t *cfg)
{
    if (nullptr == cfg)
    {
        return PYRO_PARAM_ERROR;
    }
    _cfg                  = *cfg;
    _configured_max_power = 0.0f;
    _cmd_power            = 0.0f;
    _restricted_power     = 0.0f;
    _last_energy_error    = 0.0f;
    _is_inited            = true;
    return PYRO_OK;
}

void wl_power_ctrl_t::update_energy_loop(float referee_max_power,
                                          float energy_feedback,
                                          float energy_target)
{
    float energy_error = energy_target - energy_feedback;
    float delta_error  = energy_error - _last_energy_error;

    /* PD: positive error (energy below target) → reduce P_max */
    float pd_output = _cfg.energy_kp * energy_error
                    + _cfg.energy_kd * delta_error;

    _configured_max_power = referee_max_power - pd_output;

    /* Clamp: [min_max_power, referee_max_power + cap_max_bonus] */
    if (_configured_max_power < _cfg.min_max_power)
    {
        _configured_max_power = _cfg.min_max_power;
    }
    const float upper = referee_max_power + _cfg.cap_max_bonus;
    if (_configured_max_power > upper)
    {
        _configured_max_power = upper;
    }

    _last_energy_error = energy_error;
}

void wl_power_ctrl_t::update(const wl_wheel_cmd_t cmd[MOTOR_NUM],
                              float                tau_out[MOTOR_NUM])
{
    /* ── Step 1: compute per-motor commanded power ── */
    float tau_total_cmd[MOTOR_NUM];
    float P_cmd[MOTOR_NUM];
    _cmd_power = 0.0f;

    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        tau_total_cmd[i] = cmd[i].tau_motion + cmd[i].tau_balance;
        P_cmd[i]         = predict_power(i, tau_total_cmd[i], cmd[i].omega);
        _cmd_power      += P_cmd[i];
    }

    /* ── Step 2: within budget — pass through unchanged ── */
    if (_cmd_power <= _configured_max_power)
    {
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
            tau_out[i] = tau_total_cmd[i];
        }
        _restricted_power = _cmd_power;
        return;
    }

    /* ── Step 3: proportional per-motor power allocation ──
     *   P_alloc_i = P_max * P_cmd_i / sum_j(P_cmd_j)
     *   Guard against near-zero denominator.                         */
    float P_alloc[MOTOR_NUM];
    if (_cmd_power > 1e-5f)
    {
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
            P_alloc[i] = _configured_max_power * P_cmd[i] / _cmd_power;
        }
    }
    else
    {
        const float share = _configured_max_power / static_cast<float>(MOTOR_NUM);
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
            P_alloc[i] = share;
        }
    }

    /* ── Step 4: per-motor quadratic solve with balance protection ── */
    _restricted_power = 0.0f;
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        /* Power consumed by balance torque alone.
         * If the allocated budget is ≤ this, zeroing motion is the floor. */
        const float P_balance_only = predict_power(i, cmd[i].tau_balance,
                                                       cmd[i].omega);

        if (P_alloc[i] <= P_balance_only)
        {
            /* Budget exhausted by balance — zero motion torque */
            tau_out[i] = cmd[i].tau_balance;
        }
        else
        {
            tau_out[i] = _solve_max_torque(i, cmd[i].omega,
                                           P_alloc[i],
                                           tau_total_cmd[i]);

            /* Safety: if the solver flipped the motion torque sign, clamp */
            const float tau_restricted_motion = tau_out[i] - cmd[i].tau_balance;
            if (tau_restricted_motion * cmd[i].tau_motion < 0.0f)
            {
                tau_out[i] = cmd[i].tau_balance;
            }
        }

        _restricted_power += predict_power(i, tau_out[i], cmd[i].omega);
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/* Private helpers                                                              */
/* ─────────────────────────────────────────────────────────────────────────── */

float wl_power_ctrl_t::predict_power(uint8_t motor_idx,
                                      float   tau,
                                      float   omega) const
{
    return tau * omega
         + _cfg.k1[motor_idx] * fabsf(omega)
         + _cfg.k2[motor_idx] * tau * tau
         + _cfg.k3[motor_idx];
}

float wl_power_ctrl_t::_solve_max_torque(uint8_t motor_idx,
                                          float   omega,
                                          float   P_alloc,
                                          float   tau_total_cmd) const
{
    /* Quadratic:  A*x² + B*x + C = 0
     *   A = k2[i],  B = omega,
     *   C = k1[i]*|omega| + k3[i] - P_alloc                         */
    const float C = _cfg.k1[motor_idx] * fabsf(omega)
                  + _cfg.k3[motor_idx]
                  - P_alloc;

    /* ── Degenerate: k2 ≈ 0 → linear:  omega*x + C = 0 ── */
    if (fabsf(_cfg.k2[motor_idx]) < 1e-5f)
    {
        if (fabsf(omega) < 1e-5f)
        {
            return tau_total_cmd;
        }
        const float x = -C / omega;
        return (x * tau_total_cmd >= 0.0f) ? x : 0.0f;
    }

    /* ── General quadratic ── */
    const float A     = _cfg.k2[motor_idx];
    const float B     = omega;
    const float Delta = B * B - 4.0f * A * C;

    if (Delta < 0.0f)
    {
        /* No real root — vertex is the minimum-power torque at this speed */
        return -B / (2.0f * A);
    }

    if (Delta < 1e-10f)
    {
        const float x = -B / (2.0f * A);
        return (x * tau_total_cmd >= 0.0f) ? x : 0.0f;
    }

    /* Two distinct roots — select the one sign-matched to the command.
     * If both match, take the larger magnitude (least restriction). */
    const float sqrt_delta = sqrtf(Delta);
    const float x1         = (-B + sqrt_delta) / (2.0f * A);
    const float x2         = (-B - sqrt_delta) / (2.0f * A);

    const bool x1_valid = (x1 * tau_total_cmd >= 0.0f);
    const bool x2_valid = (x2 * tau_total_cmd >= 0.0f);

    if (x1_valid && x2_valid)
    {
        return (fabsf(x1) >= fabsf(x2)) ? x1 : x2;
    }
    if (x1_valid) return x1;
    if (x2_valid) return x2;

    return -B / (2.0f * A);
}

} // namespace pyro
 
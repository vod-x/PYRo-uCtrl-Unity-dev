/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-05-07
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-07
 * @Description:
 *   Per-motor power control for wheel-legged chassis (2 wheel motors).
 *
 *   ── Torque decomposition ──────────────────────────────────────────────────
 *   Each wheel motor torque command is split into two orthogonal components:
 *
 *     tau_total_i = tau_motion_i  +  tau_balance_i
 *
 *   • tau_motion_i  (Uspeed ± Uyaw in HKUST notation)
 *       Torque used for forward velocity tracking and yaw rotation.
 *       This component IS subject to power limiting.
 *
 *   • tau_balance_i  (Uelse = Upitch + Uleg in HKUST notation)
 *       Torque used to maintain body pitch balance and leg length control.
 *       This component is ALWAYS passed through unchanged regardless of
 *       the power budget, because zeroing it causes the robot to fall.
 *
 *   ── Power model (per motor, no RLS — calibrate k1/k2/k3 manually) ────────
 *     P_i = tau_total_i * omega_i
 *         + k1 * |omega_i|          (speed-dependent friction / iron loss)
 *         + k2 * tau_total_i²       (copper loss)
 *         + k3 / MOTOR_NUM          (static loss share)
 *
 *   ── Power model (per-motor independent coefficients) ─────────────────────
 *     P_i = tau_i * omega_i  +  k1[i] * |omega_i|  +  k2[i] * tau_i²  +  k3[i]
 *   k1[i], k2[i], k3[i] are calibrated individually for each wheel motor.
 *
 *   ── Restriction algorithm (per-motor independent) ─────────────────────────
 *   tau_motion_i is treated as an atomic unit (Uspeed ± Uyaw combined).
 *   When sum(P_cmd_i) > P_max, budget is allocated proportionally:
 *     P_alloc_i = P_max * P_cmd_i / sum_j(P_cmd_j)
 *
 *   For each motor, the maximum admissible total torque is found by solving:
 *     k2[i] * x²  +  omega_i * x  +  ( k1[i]*|omega_i| + k3[i] - P_alloc_i )  =  0
 *   selecting the root sign-matched to tau_total_cmd_i.
 *
 *   Balance protection: if P_alloc_i ≤ P(tau_balance_i, omega_i), the motion
 *   torque is zeroed and tau_balance_i is passed through unchanged.
 *
 *   ── Energy loop (optional) ────────────────────────────────────────────────
 *   A PD controller on capacitor/buffer energy dynamically adjusts P_max.
 *   Call update_energy_loop() every cycle, or bypass with set_max_power().
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#ifndef __PYRO_WL_POWER_CTRL_H__
#define __PYRO_WL_POWER_CTRL_H__

#include "pyro_core_def.h"
#include <cstdint>

namespace pyro
{

/* ─── Configuration ──────────────────────────────────────────────────────── */

/* Configuration parameters for the wheel-legged per-motor power controller.
   k1, k2, k3 must be calibrated for the same torque / angular-velocity unit
   system used by the wheel motor driver (no online RLS fitting). */
struct wl_power_ctrl_cfg_t
{
    /* Per-motor velocity-related loss coefficient  [ W·s/rad ] :  k1[i] * |omega|
       Index matches motor array convention: [R = 0]  [L = 1]              */
    float k1[2];
    /* Per-motor torque-squared (copper) loss coefficient  [ W / torque_unit² ] */
    float k2[2];
    /* Per-motor static loss  [ W ]                                          */
    float k3[2];
    /* Energy-loop PD gains (positive value reduces P_max when energy is low) */
    float energy_kp;
    float energy_kd;
    /* Minimum configured max power — safety floor  [ W ] */
    float min_max_power;
    /* Maximum extra power the capacitor may contribute above the referee limit [ W ] */
    float cap_max_bonus;
};

/* ─── Per-wheel input command ─────────────────────────────────────────────── */

/*
 * Per-wheel command for the power controller.
 *
 * Caller must decompose the wheel torque into the two components BEFORE
 * calling update().  For the wheel-legged chassis:
 *
 *   tau_balance = LQR pitch/beta stabilisation terms
 *                 = gain[4]*(beta_ref - beta) + gain[5]*(d_beta_ref - d_beta)
 *                 (scaled to motor torque units, sign-matched for each wheel)
 *
 *   tau_motion  = all remaining LQR terms + yaw feed-forward
 *                 = T_w_total - tau_balance_part  ±  T_w_gain_yaw
 *
 *   omega       = wheel angular velocity (rad/s), sign-matched to torque
 *                 Right wheel:  +get_current_rotate()
 *                 Left  wheel:  -get_current_rotate()
 */
struct wl_wheel_cmd_t
{
    /* Motion torque — forward speed tracking + yaw ( CAN be limited ) */
    float tau_motion;
    /* Balance torque — pitch/beta + leg length  ( NEVER limited ) */
    float tau_balance;
    /* Current wheel angular velocity (rad/s) */
    float omega;
};

/* ─── Power controller ────────────────────────────────────────────────────── */

/*
 * Per-motor power controller for the wheel-legged chassis.
 *
 * Indices convention (matches wl_chassis_t): [R = 0]  [L = 1]
 *
 * Typical call sequence each 1 ms control cycle:
 *   1. Fill wl_wheel_cmd_t[2] with decomposed torques and current speeds.
 *   2. update_energy_loop()  — refresh P_max from energy feedback.
 *      (Or set_max_power() to bypass.)
 *   3. update()              — compute restricted tau_out[2].
 *   4. Pass tau_out[R] / tau_out[L] to send_torque() (with chassis sign fix).
 */
class wl_power_ctrl_t
{
public:
    static constexpr uint8_t MOTOR_NUM = 2;

    /*
     * @description: Initialize. Must be called before any other method.
     * @param cfg   : non-null pointer to configuration
     * @return PYRO_OK / PYRO_PARAM_ERROR
     */
    status_t init(const wl_power_ctrl_cfg_t *cfg);

    /*
     * @description:
     *   Energy-loop update. Adjusts configured_max_power via PD control on
     *   (energy_target - energy_feedback).  Call once per cycle before update().
     *   Pass equal values for both energy arguments to freeze the correction.
     * @param referee_max_power : chassis power limit reported by referee (W)
     * @param energy_feedback   : capacitor or buffer energy (same unit)
     * @param energy_target     : desired energy setpoint
     */
    void update_energy_loop(float referee_max_power,
                            float energy_feedback,
                            float energy_target);

    /*
     * @description:
     *   Core power-limiting step.
     *   Power is predicted per-motor using (tau_motion + tau_balance) and omega.
     *   If within budget, tau_out = tau_motion + tau_balance (unchanged).
     *   If over budget, P_max is allocated proportionally to each motor and a
     *   per-motor quadratic is solved for the restricted total torque.
     *   tau_balance is ALWAYS preserved unchanged (balance protection).
     * @param cmd     : array of per-wheel commands [R=0, L=1]
     * @param tau_out : restricted total torque output [R=0, L=1]
     *                  Pass directly to send_torque() (with chassis sign fix).
     */
    void update(const wl_wheel_cmd_t cmd[MOTOR_NUM],
                float                tau_out[MOTOR_NUM]);

    /* Override P_max directly, bypassing the energy loop */
    void set_max_power(float max_power) { _configured_max_power = max_power; }

    /* Telemetry accessors */
    float get_configured_max_power() const { return _configured_max_power; }
    float get_cmd_power()            const { return _cmd_power; }
    float get_restricted_power()     const { return _restricted_power; }

    /*
     * @description:
     *   Predict mechanical + electrical power for one motor.
     *   P = tau*omega + k1[i]*|omega| + k2[i]*tau² + k3[i]
     * @param motor_idx : 0 = right, 1 = left
     * @param tau       : total wheel torque (N·m, or motor torque unit)
     * @param omega     : wheel angular velocity (rad/s)
     * @return          : estimated power (W)
     */
    float predict_power(uint8_t motor_idx, float tau, float omega) const;

private:

    /*
     * Solve  k2[i]*x² + omega*x + (k1[i]*|omega| + k3[i] - P_alloc) = 0
     * for the restricted total torque of motor motor_idx.
     * Sign is matched to tau_total_cmd; returns vertex on no real root.
     */
    float _solve_max_torque(uint8_t motor_idx,
                            float   omega,
                            float   P_alloc,
                            float   tau_total_cmd) const;

    wl_power_ctrl_cfg_t _cfg;

    float _configured_max_power;
    float _cmd_power;
    float _restricted_power;
    float _last_energy_error;
    bool  _is_inited;
};

} // namespace pyro

#endif /* __PYRO_WL_POWER_CTRL_H__ */

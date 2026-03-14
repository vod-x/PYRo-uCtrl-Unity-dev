/*
 * @Author: vod-x vod_x@outlook.com
 * @Date: 2026-03-08 15:12:51
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-14 15:26:06
 * @FilePath: \Wheel-Legged-Robot\embedded_system\PYRo\Algorithm\Common\pyro_algo_common.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pyro_algo_common.h"

namespace pyro
{
float wrap2pi_f32(float input)
{

    float ret = fmodf(input, 2 * PI);
    if (ret >= PI) {
        ret -= 2.0f * PI;
    } else if (ret < -PI) {
        ret += 2.0f * PI;
    }
    return ret;
    
}

float radps_to_rpm(const float radps)
{
    return radps * 9.5492966f;
}

float calculate_angle_diff(float current, float target)
{
    const float diff = std::fabs(current - target);
    return std::fmin(diff, 2 * PI - diff);
}

// 霍纳法则多项式计算 (最高次项在前)
float evaluate_polynomial(const float x, const float *coeffs,
                           const uint32_t degree)
{
    float result = coeffs[0];
    for (uint32_t i = 1; i <= degree; ++i)
    {
        result = result * x + coeffs[i];
    }
    return result;
}

float mps_to_rpm(const float mps, const float radius)
{
    if (radius < 1e-4f)
        return 0.0f;
    return (mps / radius) * 9.5492966f;
}

float loop_fp32_constrain(float val, const float min_val, const float max_val)
{
    const float len = max_val - min_val;
    if (len < 1e-6f)
        return val;
    while (val > max_val)
        val -= len;
    while (val < min_val)
        val += len;
    return val;
}
float fp32_constrain(float val, const float min_val, const float max_val)
{
    if (val > max_val)
        return max_val;
    if (val < min_val)
        return min_val;
    return val;

} 
}
// namespace pyro
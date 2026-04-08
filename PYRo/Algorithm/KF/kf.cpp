/*
 * @Author: vod-x vod_x@outlook.com
 * @Date: 2026-04-06 14:28:05
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-04-06 16:05:50
 * @FilePath: \Wheel-Legged-Robot\embedded_system\PYRo\Algorithm\KF\kf.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "kf.h"

#include <cmath>
#include <cstring>

using namespace pyro;
using namespace arm_cmsis_dsp;

kf_t::kf_t(uint8_t x_size, uint8_t u_size, uint8_t z_size, uint8_t w_size)
	/* Pre-allocate all matrices in the member initializer list.
	 * Dimensions are fixed at construction time and do not change
	 * throughout the object lifetime, avoiding heap fragmentation. */
	: _x_size(x_size),
	  _u_size(u_size),
	  _z_size(z_size),
	  _w_size(w_size),
	  _is_init(false),
	_mat_A(x_size, x_size),
	_mat_B(x_size, u_size),
	_mat_H(z_size, x_size),
	_mat_G(x_size, w_size),
	_mat_Q(w_size, w_size),
	_mat_R(z_size, z_size),
	_mat_K(x_size, z_size),
	_mat_P(x_size, x_size),
	_mat_P_minus(x_size, x_size),
	_vec_xhat(x_size, 1),
	_vec_xhat_minus(x_size, 1),
	_vec_z(z_size, 1),
	_vec_u(u_size, 1),
	_mat_I(x_size, x_size),
	_tmp_x_1(x_size, 1),
	_tmp_x_2(x_size, 1),
	_tmp_z_1(z_size, 1),
	_tmp_xx_1(x_size, x_size),
	_tmp_xx_2(x_size, x_size),
	_tmp_xx_3(x_size, x_size),
	_mat_Gt(w_size, x_size),
	_mat_Ht(x_size, z_size),
	_mat_Kt(z_size, x_size),
	_mat_S(z_size, z_size),
	_mat_S_inv(z_size, z_size),
	_tmp_xz_1(x_size, z_size),
	_tmp_xz_2(x_size, z_size),
	_tmp_xw_1(x_size, w_size)
{
}

void kf_t::fill_mat(mat &matrix, const float *data)
{
	/* Copy row-major flat array into dsppp matrix element by element.
	 * The dsppp Matrix stores data contiguously in row-major order,
	 * so a linear index is sufficient. */
	const int total = matrix.rows() * matrix.columns();
	for (int i = 0; i < total; ++i)
	{
		matrix[i] = data[i];
	}
}

void kf_t::fill_scalar(mat &matrix, float value)
{
	/* Broadcast a single scalar value to every element of the matrix.
	 * Used for zero-initialization (value = 0.0f) of K and P_minus. */
	const int total = matrix.rows() * matrix.columns();
	for (int i = 0; i < total; ++i)
	{
		matrix[i] = value;
	}
}

void kf_t::assign_vector(mat &vector, const float *data)
{
	/* Copy caller-provided vector data into an internal column vector.
	 * `vector` is expected to be shape (N x 1). */
	const int len = vector.rows();
	for (int i = 0; i < len; ++i)
	{
		vector(i, 0) = data[i];
	}
}

void kf_t::copy_vector(mat &dst, const mat &src)
{
	/* Explicit element-wise copy for column vectors to avoid dsppp copy path
	 * that requires deleted Vector_Base copy constructor in this toolchain. */
	const int len = dst.rows();
	for (int i = 0; i < len; ++i)
	{
		dst(i, 0) = src(i, 0);
	}
}

void kf_t::clear_vector(mat &vector)
{
	/* Clear an internal column vector to zero.
	 * Used in init() to reset xhat / xhat_minus / u / z. */
	const int len = vector.rows();
	for (int i = 0; i < len; ++i)
	{
		vector(i, 0) = 0.0f;
	}
}

void kf_t::set_identity(mat &matrix)
{
	/* Write 1 on the main diagonal and 0 elsewhere.
	 * Requires a square matrix; used to initialize P and I. */
	for (int r = 0; r < matrix.rows(); ++r)
	{
		for (int c = 0; c < matrix.columns(); ++c)
		{
			matrix(r, c) = (r == c) ? 1.0f : 0.0f;
		}
	}
}

bool kf_t::validate_covariance_data(const float *data, uint8_t n)
{
	/* Basic runtime checks for covariance matrix data:
	 * 1) all elements finite
	 * 2) symmetric within tolerance
	 * 3) diagonal non-negative
	 * Note: this does not fully prove positive semi-definiteness. */
	if (data == nullptr)
	{
		return false;
	}

	constexpr float sym_eps = 1e-5f;
	for (uint8_t r = 0; r < n; ++r)
	{
		const float diag = data[r * n + r];
		if (!std::isfinite(diag) || diag < 0.0f)
		{
			return false;
		}

		for (uint8_t c = 0; c < n; ++c)
		{
			const float a = data[r * n + c];
			if (!std::isfinite(a))
			{
				return false;
			}

			const float b = data[c * n + r];
			const float scale = std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
			if (std::fabs(a - b) > sym_eps * scale)
			{
				return false;
			}
		}
	}

	return true;
}

arm_status kf_t::inverse_matrix(mat &src, mat &dst)
{
	/* Bridge the dsppp matrix storage to the CMSIS-DSP C API.
	 * arm_mat_inverse_f32 performs LU decomposition in-place;
	 * src and dst may alias as long as they point to the same buffer,
	 * but here we use separate matrices to keep src readable after inversion.
	 * Returns ARM_MATH_SINGULAR if the matrix is not invertible. */
	arm_matrix_instance_f32 src_mat;
	arm_matrix_instance_f32 dst_mat;
	arm_mat_init_f32(&src_mat, static_cast<uint16_t>(src.rows()), static_cast<uint16_t>(src.columns()), &src[0]);
	arm_mat_init_f32(&dst_mat, static_cast<uint16_t>(dst.rows()), static_cast<uint16_t>(dst.columns()), &dst[0]);
	return arm_mat_inverse_f32(&src_mat, &dst_mat);
}

status_t kf_t::init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data)
{
	return init_impl(A_data, B_data, H_data, G_data, Q_data, R_data, nullptr, nullptr);
}

status_t kf_t::init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
					float *x0_data)
{
	return init_impl(A_data, B_data, H_data, G_data, Q_data, R_data, x0_data, nullptr);
}

status_t kf_t::init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
					std::nullptr_t, float *P0_data)
{
	return init_impl(A_data, B_data, H_data, G_data, Q_data, R_data, nullptr, P0_data);
}

status_t kf_t::init(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
					float *x0_data, float *P0_data)
{
	return init_impl(A_data, B_data, H_data, G_data, Q_data, R_data, x0_data, P0_data);
}

status_t kf_t::init_impl(float *A_data, float *B_data, float *H_data, float *G_data, float *Q_data, float *R_data,
					 const float *x0_data, const float *P0_data)
{
	/* 1. Validate input: all matrix data pointers must be non-null */
	if (A_data == nullptr || B_data == nullptr || H_data == nullptr || G_data == nullptr || Q_data == nullptr || R_data == nullptr)
	{
		return PYRO_PARAM_ERROR;
	}

	/* 2. Guard against repeated initialization */
	if (_is_init)
	{
		return PYRO_ALREADY_INIT;
	}

	/* 2.1 Validate custom initial covariance data when provided */
	if ((P0_data != nullptr) && (!validate_covariance_data(P0_data, _x_size)))
	{
		return PYRO_PARAM_ERROR;
	}

	/* 3. Load system matrices from caller-supplied row-major data arrays
	 *    A (x_size x x_size) : state transition matrix
	 *    B (x_size x u_size) : control input matrix
	 *    H (z_size x x_size) : measurement (observation) matrix
	 *    G (x_size x w_size) : noise transition matrix
	 *    Q (w_size x w_size) : process noise covariance
	 *    R (z_size x z_size) : measurement noise covariance */
	fill_mat(_mat_A, A_data);
	fill_mat(_mat_B, B_data);
	fill_mat(_mat_H, H_data);
	fill_mat(_mat_G, G_data);
	fill_mat(_mat_Q, Q_data);
	fill_mat(_mat_R, R_data);

	/* 4. Initialize posterior state estimate x_{0|0}
	 *    default: zero vector
	 *    custom : use user-provided x0_data */
	if (x0_data == nullptr)
	{
		clear_vector(_vec_xhat);
	}
	else
	{
		assign_vector(_vec_xhat, x0_data);
	}

	/* Keep prior state cache consistent at startup */
	copy_vector(_vec_xhat_minus, _vec_xhat);

	/* 5. Initialize input/measurement vectors */
	clear_vector(_vec_u);
	clear_vector(_vec_z);

	/* 6. Initialize working matrices
	 *    K      : Kalman gain, starts at zero
	 *    P_minus: prior error covariance, starts at zero
	 *    I      : identity matrix, used in posterior covariance update
	 *    P      : posterior error covariance
	 *             default: identity matrix
	 *             custom : user-provided P0_data */
	fill_scalar(_mat_K, 0.0f);
	fill_scalar(_mat_P_minus, 0.0f);
	set_identity(_mat_I);
	if (P0_data == nullptr)
	{
		set_identity(_mat_P);
	}
	else
	{
		fill_mat(_mat_P, P0_data);
	}

	/* 7. Mark filter as initialized to allow update() to proceed */
	_is_init = true;
	return PYRO_OK;
}

status_t kf_t::update(float *measure_vec, float *control_vec, float *estimated_ret)
{
	/* 1. Validate input pointers */
	if (measure_vec == nullptr || control_vec == nullptr || estimated_ret == nullptr)
	{
		return PYRO_PARAM_ERROR;
	}

	/* 2. Ensure init() has been called before the first update */
	if (!_is_init)
	{
		return PYRO_NOT_FOUND;
	}

	/* 3. Copy current measurement z_k and control input u_k into internal vectors */
	assign_vector(_vec_z, measure_vec);
	assign_vector(_vec_u, control_vec);

	/* 4. Predict step — project state ahead
	 *    Prior state estimate:
	 *        x_{k}^- = A * x_{k-1} + B * u_{k-1}
	 *    Temporary variables:
	 *      _tmp_x_1 (x_size x 1) = A * x_{k-1}
	 *      _tmp_x_2 (x_size x 1) = B * u_{k-1} */
	_tmp_x_1 = _mat_A * _vec_xhat;
	_tmp_x_2 = _mat_B * _vec_u;
	_vec_xhat_minus = _tmp_x_1 + _tmp_x_2;

	/* 5. Predict step — project error covariance ahead
	 *    Prior error covariance:
	 *        P_{k}^- = A * P_{k-1} * A^T + G * Q * G^T
	 *    Temporary variables:
	 *      _tmp_xx_1 (x_size x x_size) = A * P_{k-1}, reused for G*Q*G^T
	 *      _tmp_xx_2 (x_size x x_size) = A^T
	 *      _tmp_xw_1 (x_size x w_size) = G * Q
	 *      _mat_Gt   (w_size x x_size) = G^T */
	_tmp_xx_1 = _mat_A * _mat_P;
	_tmp_xx_2 = _mat_A.transpose();
	_mat_P_minus = _tmp_xx_1 * _tmp_xx_2;
	_tmp_xw_1 = _mat_G * _mat_Q;
	_mat_Gt = _mat_G.transpose();
	_tmp_xx_1 = _tmp_xw_1 * _mat_Gt;
	_mat_P_minus = _mat_P_minus + _tmp_xx_1;

	/* 6. Update step — compute innovation covariance S
	 *    S = H * P_{k}^- * H^T + R
	 *    S represents total uncertainty in measurement space.
	 *    Temporary variables:
	 *      _mat_Ht  (x_size x z_size) = H^T
	 *      _tmp_z_1 (z_size x 1)      = H * x_k^- (predicted measurement)
	 *      _tmp_xz_1(x_size x z_size) = P_k^- * H^T */
	_mat_Ht = _mat_H.transpose();
	_tmp_z_1 = _mat_H * _vec_xhat_minus;
	_tmp_xz_1 = _mat_P_minus * _mat_Ht;
	_mat_S = _mat_H * _tmp_xz_1 + _mat_R;

	/* 7. Update step — compute S^{-1} for Kalman gain calculation
	 *    Returns ARM_MATH_SINGULAR if S is not invertible (e.g. rank-deficient R) */
	arm_status math_ret = inverse_matrix(_mat_S, _mat_S_inv);
	CHECK_ARM_MATH_RET(math_ret);

	/* 8. Update step — compute optimal Kalman gain
	 *    K_{k} = P_{k}^- * H^T * S^{-1} */
	_mat_K = _tmp_xz_1 * _mat_S_inv;

	/* 9. Update step — correct state estimate with measurement residual
	 *    x_{k} = x_{k}^- + K_{k} * (z_{k} - H * x_{k}^-)
	 *    The term (z_{k} - H * x_{k}^-) is the innovation (measurement residual) */
	_vec_xhat = _vec_xhat_minus + _mat_K * (_vec_z - _tmp_z_1);

	/* 10. Update step — update posterior error covariance using Joseph form
	 *     A_j = (I - K_{k} * H)
	 *     P_{k} = A_j * P_{k}^- * A_j^T + K_{k} * R * K_{k}^T
	 *     Temporary variables:
	 *      _tmp_xx_3 (x_size x x_size) = A_j
	 *      _tmp_xx_1 (x_size x x_size) = A_j * P_k^- then reused as K*R*K^T
	 *      _tmp_xx_2 (x_size x x_size) = A_j^T
	 *      _tmp_xz_2 (x_size x z_size) = K * R
	 *      _mat_Kt   (z_size x x_size) = K^T
	 *     This form better preserves symmetry and positive semi-definiteness. */
	_tmp_xx_3 = _mat_I - _mat_K * _mat_H;
	_tmp_xx_1 = _tmp_xx_3 * _mat_P_minus;
	_tmp_xx_2 = _tmp_xx_3.transpose();
	_mat_P = _tmp_xx_1 * _tmp_xx_2;

	_tmp_xz_2 = _mat_K * _mat_R;
	_mat_Kt = _mat_K.transpose();
	_tmp_xx_1 = _tmp_xz_2 * _mat_Kt;
	_mat_P = _mat_P + _tmp_xx_1;

	/* 11. Write posterior state estimate back to caller's output buffer */
	for (int i = 0; i < _x_size; ++i)
	{
		estimated_ret[i] = _vec_xhat(i, 0);
	}

	return PYRO_OK;
}

status_t kf_t::get_state(float *out) const
{
	if (out == nullptr)
	{
		return PYRO_PARAM_ERROR;
	}
	if (!_is_init)
	{
		return PYRO_NOT_FOUND;
	}
	for (int i = 0; i < _x_size; ++i)
	{
		out[i] = _vec_xhat(i, 0);
	}
	return PYRO_OK;
}

status_t kf_t::reset()
{
	if (!_is_init)
	{
		return PYRO_NOT_FOUND;
	}

	/* Reset state to zero, covariance to identity */
	clear_vector(_vec_xhat);
	copy_vector(_vec_xhat_minus, _vec_xhat);
	set_identity(_mat_P);
	fill_scalar(_mat_K, 0.0f);
	fill_scalar(_mat_P_minus, 0.0f);
	clear_vector(_vec_u);
	clear_vector(_vec_z);

	return PYRO_OK;
}

status_t kf_t::reset(float *x0_data, float *P0_data)
{
	if (!_is_init)
	{
		return PYRO_NOT_FOUND;
	}
	if (x0_data == nullptr || P0_data == nullptr)
	{
		return PYRO_PARAM_ERROR;
	}
	if (!validate_covariance_data(P0_data, _x_size))
	{
		return PYRO_PARAM_ERROR;
	}

	assign_vector(_vec_xhat, x0_data);
	copy_vector(_vec_xhat_minus, _vec_xhat);
	fill_mat(_mat_P, P0_data);
	fill_scalar(_mat_K, 0.0f);
	fill_scalar(_mat_P_minus, 0.0f);
	clear_vector(_vec_u);
	clear_vector(_vec_z);

	return PYRO_OK;
}

status_t kf_t::reset(float *x0_data)
{
	if (!_is_init)
	{
		return PYRO_NOT_FOUND;
	}
	if (x0_data == nullptr)
	{
		return PYRO_PARAM_ERROR;
	}

	assign_vector(_vec_xhat, x0_data);
	copy_vector(_vec_xhat_minus, _vec_xhat);
	fill_scalar(_mat_K, 0.0f);
	fill_scalar(_mat_P_minus, 0.0f);
	clear_vector(_vec_u);
	clear_vector(_vec_z);

	return PYRO_OK;
}

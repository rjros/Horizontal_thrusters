/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RateControl.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	//Multiply the inertia tensor
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}
	return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}

// GEOMETRIC CONTROLLER SECTION ///

void RateControl::setInertiaMatrix(const matrix::Matrix3f &Inertia)
{
	// In the case it needs to be updated
	_Ib=Inertia;


}

void RateControl::setAttitudeSetpoint(const geometric_setpoint_s &thrust_vect_sp)
{
	_Rd = Matrix3f(thrust_vect_sp.rd);
	_Wd = Vector3f(thrust_vect_sp.wd);
	// _Wd.print();
	_Wd_dot = Vector3f(thrust_vect_sp.wd_dot);

}

void RateControl::setGeometricAttitudeGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D)
{
	_gain_geom_p = P;
	_gain_geom_i = I;
	_gain_geom_d = D;
	_c2 =1.0f;

	// ct may be changed in the parameters

}

void RateControl::setTorqueLimit(const float x_torque_max, const float y_torque_max, const float z_torque_max)
{
	_x_torque_max = x_torque_max;
	_y_torque_max = y_torque_max;
	_z_torque_max = z_torque_max;

}


void RateControl::setAttitudeStates(const matrix::Quaternionf &attitude, const matrix::Vector3f &rates,const matrix::Vector3f &angular_accel)
{
	_R = attitude;
	_rates = rates;
	// _rates.print();
	_angular_acceleration = angular_accel;

}



// Vector3f RateControl::update_mc(const Vector3f &rate, const SquareMatrix<float,3> &Inertia ,const Vector3f &rate_sp, const Vector3f &angular_accel,
// 			     const float dt, const bool landed)
// {
// 	// angular rates error
// 	//acceleration desired?
// 	Vector3f rate_error = rate_sp - rate;
// 	float wb[9]={0, -rate(2), rate(1),
// 		 rate(2),   0, -rate(0),
// 		-rate(1), rate(0), 0};
// 	SquareMatrix<float,3> S_wb(wb);
// 	SquareMatrix <float,3> I_b=Inertia;

// 	// PID control with feed forward
// 	//Multiply the inertia tensor
// 	const Vector3f rate_accel =_gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

// 	Vector3f torque = I_b*rate_accel*2 + S_wb*I_b*rate;

// 	// torque.print();

// 	// update integral only if we are not landed
// 	if (!landed) {
// 		updateIntegral(rate_error, dt);
// 	}
// 	// Inertia.print();
// 	torqueNormalization(torque);
// 	return torque;
// }

Vector3f RateControl::update_mc(const float dt, const bool landed)
{
	geometricController(dt,landed);
	torqueNormalization();
	return _torque;

}

void RateControl::geometricController(const float dt,const bool landed)
{

	// Everything in the world frame
	_Wd(0) = PX4_ISFINITE(_Wd(0) ) ?_Wd(0)  : _rates(0);
	_Wd(1) = PX4_ISFINITE(_Wd(1)) ?_Wd(1)  : _rates(1);
	_Wd(2) = PX4_ISFINITE(_Wd(2))  ?_Wd(2)  : _rates(2);

	// _Wd.print();



	_Wd_dot(0) = PX4_ISFINITE(_Wd_dot(0) ) ?_Wd_dot(0)  : _angular_acceleration(0);
	_Wd_dot(1) = PX4_ISFINITE(_Wd_dot(1)) ? _Wd_dot(1)  : _angular_acceleration(1);
	_Wd_dot(2) = PX4_ISFINITE(_Wd_dot(2))  ?_Wd_dot(2)  : _angular_acceleration(2);



	Dcmf RdtR = _Rd.transpose() * _R;
	Vector3f e_R = 0.5f * Dcmf(RdtR-RdtR.transpose()).vee();
	Vector3f e_W = _rates - _R.transpose()*_Rd*_Wd;


	_torque = -e_R.emult(_gain_geom_p) - e_W.emult(_gain_geom_d) - _geom_int\
		+ Vector3f(_R.transpose()*_Rd*_Wd).hat()*_Ib*_R.transpose()*_Rd*_Wd \
		+ _Ib* _R.transpose()*_Rd*_Wd_dot;
	// update integral only if we are not landed
	// _torque.print();
	// update integral only if we are not landed
	if (!landed) {
		updateIntegralGeometric(e_R,e_W ,dt);
	}
	torqueNormalization();

}


void RateControl::updateIntegralGeometric(Vector3f &att_error, Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		// float i_factor = rate_error(i) / math::radians(400.f);
		// i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = rate_error(i) + _c2 * _gain_geom_i(i) *att_error(i)*dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_geom_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}


void RateControl::torqueNormalization()
{
	// Toque setpoint normalization
	_torque(0) = PX4_ISFINITE(_torque(0)) ?  _torque(0) / _x_torque_max : 0.0f;
	_torque(1) = PX4_ISFINITE(_torque(1)) ? _torque(1) / _y_torque_max : 0.0f;
	_torque(2) = PX4_ISFINITE(_torque(2)) ? _torque(2) / _z_torque_max : 0.0f;

	_torque(0) = math::constrain(_torque(0), -1.0f, 1.0f);\
	_torque(1) = math::constrain(_torque(1), -1.0f, 1.0f);
	_torque(2) = math::constrain(_torque(2), -1.0f, 1.0f);


}


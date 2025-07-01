/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 * Modifications for suport of geometric controller SE(3)
 * @author Ricardo Rosales Martinez
 */


#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>

#include <geo/geo.h>
#include <bitset>
#include<iostream>


using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}
//Planar Gains//
void PositionControl::setPlanarPositionGains(const Vector3f &P)
{
	//Use values from the users parameters,this depends on number of fans
	_gain_planar_pos_p = P;
}
void PositionControl::setPlanarVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	//Use values from the users parameters,this depends on number of fans
	_gain_planar_vel_p = P;
	_gain_planar_vel_i = I;
	_gain_planar_vel_d = D;
}
//Planar Gains End//

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setPlanarThrustLimits(const float min, const float max,const float planar_threshold)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_planar_thr_min = math::max(min, 10e-4f);
	_lim_planar_thr_max = max;
	_planar_threshold = planar_threshold;
}


void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
	_rates = states.rates;
	_attitude = states.attitude;
	_R =_attitude; // Rotation matrix of the Body frame
	_CA_mode = states.CA_mode;

}


//// GEOEMETRIC CONTROLLER FUNCTIONS ////
void PositionControl::setMass(const float vehicle_mass)
{
	_mass=vehicle_mass;
}


void PositionControl::setGeometricThrustLimits(const float x_thrust_max,const float y_thrust_max,const float z_thrust_max)
{
	_max_thrust_x = x_thrust_max;
	_max_thrust_y = y_thrust_max;
	_max_thrust_z = z_thrust_max;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;


}


void PositionControl::setGeometricPositionGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_geom_p = P;
	_gain_geom_i = I;
	_gain_geom_d = D;
}


void PositionControl::setGeometricPositionThrusterGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_geom_thr_p = P;
	_gain_geom_thr_i = I;
	_gain_geom_thr_d = D;
}


bool PositionControl::update(const float dt, const int vectoring_att_mode,bool planar_flight)
{
	bool valid = _inputValid();

	if (valid) {

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;

	bool distance_flag=false;
	float error_xy=sqrt(pow((_pos_sp(0) - _pos(0)),2)+pow((_pos_sp(1) - _pos(1)),2));
	distance_flag= (error_xy>=_planar_threshold)?true:false;
	//Distance becomes nan during manual motion
	bool moving_flag=false;
	moving_flag=!PX4_ISFINITE(error_xy)?true:false;
	//Conditions for planar motion
	//Vectoring mode on or off only
	planar_flag=(planar_flight||distance_flag||moving_flag||true);

	switch (vectoring_att_mode) {


	case 1:
		_autoPlanar_positionControl(dt,_yaw_sp);
		break;

	case 2:

		_planar_X_positionControl(dt,_yaw_sp);
		_planar_X_velocityControl(dt,_yaw_sp);
		break;
	case 3:
		_positionControl();
		_velocityControl(dt);
		break;//here

	default:
		_positionControl();
		_velocityControl(dt);
		}
	}


	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

//// GEOEMETRIC CONTROLLER FUNCTIONS ////

//// CUSTOM PARAMETERS FOR PLANAR FLIGHT MODE ////

bool PositionControl::updateGeometric(const float dt, const int vectoring_att_mode)
{
	bool valid = _inputValid();
	bool acc_valid =false;

	if (valid) {
	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
	_R_yaw = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw_sp}};

	// PX4_INFO("Yaw setpoint %f",double(_yaw_sp));

	// _geometricControl(dt);
	// _geometric_X_thrusters(dt);
	switch (vectoring_att_mode) {


	case 1:
		_geometricAuto(dt);
		break;

	case 2:
		// Testing mode
		//TODO Let user select how to fill available modes for RC change option
		_geometric_X_thrusters(dt);
		break;
	case 3:

		_geometricControl(dt);
		break;//here

	default:
		_geometricControl(dt);
		}


	acc_valid = _thrust_sp.isAllFinite();
	}


	return valid && acc_valid;

}

//// CUSTOM PARAMETERS FOR GEOMETRIC CONTROLLER ////
void PositionControl::_positionControl()
{	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p); // x,y,z different when using fans and roll and pitch...
	// PX4_INFO("Position %f velocity %f acceleration %f", double(_pos_sp(2)),double(_vel_sp(2)),double(_acc_sp(2)));

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// PX4_INFO("Velocity after %f", double(_vel_sp(2)));

	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);
	// vel_sp_position.print();

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
	// PX4_INFO("Position setpoint %f %f %f",(double)_pos_sp(0),(double)_pos_sp(1),(double)_pos_sp(2));
}

void PositionControl::_velocityControl(const float dt)
{

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	// Estimate the optimal tilt angle and direction to conteract the wind
	// Prioritize vertical control while keeping a horizontal margin
	//Mode dependant with additional actuators is not needed
	Vector2f thrust_sp_xy(_thr_sp);
	float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);

	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / (_hover_thrust*2));
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity controld
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
	// _vel_int.print();
	// _thr_sp.print();
	// _vel_int.print();
	// PX4_INFO("Thrust  %f %f %f",(double)_thr_sp(0),(double)_thr_sp(1),(double)_thr_sp(2));
}


/////Simple controller/////

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	// PX4_INFO("Acceleration setpoint %f %f %f",(double)_acc_sp(0),(double)_acc_sp(1),(double)_acc_sp(2));

	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);

	_thr_sp = body_z * collective_thrust;
}
/////Simple controller/////


void PositionControl::_geometricAuto(const float dt)
{
	//create temp variables since the modes have different gains
	Vector3f vel_sp = _vel_sp;
	Vector3f pos_sp = _pos_sp;

	Vector3f pos_error = pos_sp - _pos;
	Vector3f vel_sp_position = pos_error.emult(Vector3f{1,1,1});

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(vel_sp, vel_sp_position);

	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Check the sp direction in the body frame to select the mode
	//If X +, check what mode is needed {X+,X-,Y+,Y-}
	//If Y +, check what mode is needed {X+,X-,Y+,Y-}
	Vector3f vel_sp_body=_R.transpose() * vel_sp;
	Vector3f acc_sp_body=_R.transpose() * _acc_sp;


	int8_t sp_flags{0};
	// When acc setpoints are sent this get priority over the velocity
	if (acc_sp_body.isAllNan())
	{
		sp_flags |= (vel_sp_body(0) >= 0) ? 0b1000 : 0b0100; // 1 in the X bit for positive X
		sp_flags |= (vel_sp_body(1) >= 0) ? 0b0010 : 0b0001; // 1 in the Y bit for positive Y

	}
	else
	{
		sp_flags |= (acc_sp_body(0) >= 0) ? 0b1000 : 0b0100; // 1 in the X bit for positive X
		sp_flags |= (acc_sp_body(1) >= 0) ? 0b0010 : 0b0001; // 1 in the Y bit for positive Y
	}

	// PX4_INFO("Current diretion %d",flags);
	// check supported vehicle
	// X (+,-) supported  3
	// Only X + supported 2
	// only X - supported 3

	int CA_flags{0};
	// _CA_mode.print();
	// Check the supported mode
	CA_flags |= (_CA_mode(0)>0 ? 0b1000 : 0b0000); // Bit 3 for X+
	CA_flags |= (_CA_mode(1)>0 ? 0b0100 : 0b0000); // Bit 2 for X-
	CA_flags |= (_CA_mode(2)>0 ? 0b0010 : 0b0000); // Bit 1 for Y+
	CA_flags |= (_CA_mode(3)>0 ? 0b0001 : 0b0000); // Bit 0 for Y-


	_control_mode = (sp_flags & CA_flags);

	if (_control_mode == 0b1010 || _control_mode == 0b0110 ||
        	_control_mode == 0b1001 || _control_mode == 0b0101)
	{
		_auto_mode=1;
		// PX4_INFO("XY Thuster mode");
		_geometric_XY_thrusters(dt);
	}
	else if (_control_mode == 0b1000 || _control_mode == 0b0100)
	{
		_auto_mode=2;
		// PX4_INFO("X Thuster mode");
		_geometric_X_thrusters(dt);
	}

	else if (_control_mode == 0b0010 || _control_mode == 0b0001)
	{
		_auto_mode=3;
		// PX4_INFO("Y Thuster mode");
		_geometric_Y_thrusters(sp_flags,dt);
	}
	else {
		_auto_mode=4;
		// PX4_INFO("Tilted Thuster mode");
		_geometricControl(dt);
	}
}


void PositionControl::_geometricControl(const float dt)
{
	//Normal UAV flight using the geometric controller
	// Rotation to the body frame
	// _pos 	  = _R.transpose()* _pos;
	// _pos_sp   = _R.transpose()* _pos_sp;
	// _vel 	  = _R.transpose()* _vel;
	// _vel_sp   = _R.transpose()* _vel_sp;
	// _acc_sp   = _R.transpose()* _acc_sp;

	//Gains
	Vector3f Kp = _gain_geom_p;
	Vector3f Ki = _gain_geom_i;
	Vector3f Kd = _gain_geom_d;

	_vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

	// Vector3f acceleration_sp= _acc_sp;

	Vector3f pos_error = (_pos-_pos_sp);
	Vector3f vel_error = (_vel-_vel_sp);

	ControlMath::setZeroIfNanVector3f(pos_error);
	ControlMath::setZeroIfNanVector3f(vel_error);
	float _c1 = 1.0f;
	float _sigma = 10.0f;


	ControlMath::setZeroIfNanVector3f(_geom_int);
	for(int i=0; i<3; i++){

		float deltaI = (vel_error(i) + _c1 * pos_error(i)) * dt;

		if(PX4_ISFINITE(deltaI)){
			if( (_geom_int(i) <= -_sigma && deltaI < 0.0f) ||
					(_geom_int(i) >= _sigma && deltaI > 0.0f) ){
				deltaI = 0.0f;
			}

			_geom_int(i) += deltaI;
		}
	}


	Vector3f acc_pid =  -pos_error.emult(Kp) - vel_error.emult(Kd)
	-constrain(_geom_int, -_sigma,_sigma).emult(Ki);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_pid);

	// Vector3f g_b =  _R.transpose()*Vector3f(0.0f,0.0f,CONSTANTS_ONE_G);

	Vector3f g_b = Vector3f(0.0f,0.0f,CONSTANTS_ONE_G);


	// Work in the world frame
	// _f_b = _mass * (_acc_sp - g_b);

	_f_w = _mass * (_acc_sp - g_b);


	//_f_w = _R_yaw * _f_b;


	// Rotate to Inertial Frame
	// _pos 	  = _R* _pos;
	// _pos_sp   = _R* _pos_sp;
	// _vel 	  = _R* _vel;
	// _vel_sp   = _R* _vel_sp;
	// _acc_sp   = _R* _acc_sp;
	// _f_w = _mass * (_acc_sp - (Vector3f(0.0f,0.0f,CONSTANTS_ONE_G)));

}


// Geometric Flight Modes

void PositionControl::_geometric_XY_thrusters(const float dt)
{
	// Rotation to the body frame
	_pos 	  = _R_yaw.transpose()* _pos;
	_pos_sp   = _R_yaw.transpose()* _pos_sp;
	_vel 	  = _R_yaw.transpose()* _vel;
	_vel_sp   = _R_yaw.transpose()* _vel_sp;
	_acc_sp   = _R_yaw.transpose()* _acc_sp;

	// Gains
	Vector3f Kp = _gain_geom_thr_p;
	Vector3f Ki = _gain_geom_thr_i;
	Vector3f Kd = _gain_geom_thr_d;

	_vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);


	Vector3f pos_error = (_pos-_pos_sp);
	Vector3f vel_error = (_vel-_vel_sp);

	ControlMath::setZeroIfNanVector3f(pos_error);
	ControlMath::setZeroIfNanVector3f(vel_error);
	float _c1 = 1.0f;
	float _sigma = 10.0f;

	ControlMath::setZeroIfNanVector3f(_geom_int);
	for(int i=0; i<3; i++){

		float deltaI = (vel_error(i) + _c1 * pos_error(i)) * dt;

		if(PX4_ISFINITE(deltaI)){
			if( (_geom_int(i) <= -_sigma && deltaI < 0.0f) ||
					(_geom_int(i) >= _sigma && deltaI > 0.0f) ){
				deltaI = 0.0f;
			}

			_geom_int(i) += deltaI;
		}
	}

	Vector3f acc_pid =  -pos_error.emult(Kp) - vel_error.emult(Kd)
	-constrain(_geom_int, -_sigma,_sigma).emult(Ki);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_pid);

	Vector3f g_b =  _R_yaw.transpose()*Vector3f(0.0f,0.0f,CONSTANTS_ONE_G);

	_f_b = _mass * (_acc_sp - g_b);


	_f_w = _R_yaw * Vector3f(0.0f,0.0f,_f_b(2));

	// Rotate to Inertial Frame
	_pos 	  = _R_yaw* _pos;
	_pos_sp   = _R_yaw* _pos_sp;
	_vel 	  = _R_yaw* _vel;
	_vel_sp   = _R_yaw* _vel_sp;
	_acc_sp   = _R_yaw* _acc_sp;
}


void PositionControl::_geometric_X_thrusters(const float dt)
{

	// Vector3f acceleration_sp= _acc_sp;
	// Rotation to the body frame
	_pos 	  = _R_yaw.transpose()* _pos;
	_pos_sp   = _R_yaw.transpose()* _pos_sp;
	_vel 	  = _R_yaw.transpose()* _vel;
	_vel_sp   = _R_yaw.transpose()* _vel_sp;
	_acc_sp   = _R_yaw.transpose()* _acc_sp;

	// Gains
	Vector3f Kp = {_gain_geom_thr_p(0),_gain_geom_p(1),_gain_geom_p(2)};
	Vector3f Ki = {_gain_geom_thr_i(0),_gain_geom_i(1),_gain_geom_i(2)};
	Vector3f Kd = {_gain_geom_thr_d(0),_gain_geom_d(1),_gain_geom_d(2)};

	_vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

	Vector3f pos_error = (_pos-_pos_sp);
	Vector3f vel_error = (_vel-_vel_sp);

	ControlMath::setZeroIfNanVector3f(pos_error);
	ControlMath::setZeroIfNanVector3f(vel_error);
	float _c1 = 1.0f;
	float _sigma = 10.0f;

	ControlMath::setZeroIfNanVector3f(_geom_int);
	for(int i=0; i<3; i++){

		float deltaI = (vel_error(i) + _c1 * pos_error(i)) * dt;

		if(PX4_ISFINITE(deltaI)){
			if( (_geom_int(i) <= -_sigma && deltaI < 0.0f) ||
					(_geom_int(i) >= _sigma && deltaI > 0.0f) ){
				deltaI = 0.0f;
			}

			_geom_int(i) += deltaI;
		}
	}

	Vector3f acc_pid =  -pos_error.emult(Kp) - vel_error.emult(Kd)
	-constrain(_geom_int, -_sigma,_sigma).emult(Ki);

	// Allows takeoff
	// _acc_sp.print();
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_pid);

	Vector3f g_b =  _R_yaw.transpose()*Vector3f(0.0f,0.0f,CONSTANTS_ONE_G);


	_f_b = _mass * (_acc_sp - g_b);


	_f_w = _R_yaw * Vector3f(0.0f,_f_b(1),_f_b(2));



	// Rotate to Inertial Frame
	_pos 	  = _R_yaw* _pos;
	_pos_sp   = _R_yaw* _pos_sp;
	_vel 	  = _R_yaw* _vel;
	_vel_sp   = _R_yaw* _vel_sp;
	_acc_sp   = _R_yaw* _acc_sp;

}

void PositionControl::_geometric_Y_thrusters(const int sp_flag,const float dt)
{


	// Rotation to the body frame
	_pos 	  = _R_yaw.transpose()* _pos;
	_pos_sp   = _R_yaw.transpose()* _pos_sp;
	_vel 	  = _R_yaw.transpose()* _vel;
	_vel_sp   = _R_yaw.transpose()* _vel_sp;
	_acc_sp   = _R_yaw.transpose()* _acc_sp;

	ControlMath::setZeroIfNanVector3f(_acc_sp);

	// Gains
	Vector3f Kp = {_gain_geom_p(0),_gain_geom_thr_p(1),_gain_geom_p(2)};
	Vector3f Ki = {_gain_geom_i(0),_gain_geom_thr_i(1),_gain_geom_i(2)};
	Vector3f Kd = {_gain_geom_d(0),_gain_geom_thr_d(1),_gain_geom_d(2)};
	// Kp.print();
	// Kd.print();

	_vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

	Vector3f pos_error = (_pos-_pos_sp);
	Vector3f vel_error = (_vel-_vel_sp);

	ControlMath::setZeroIfNanVector3f(pos_error);
	ControlMath::setZeroIfNanVector3f(vel_error);
	float _c1 = 1.0f;
	float _sigma = 10.0f;

	ControlMath::setZeroIfNanVector3f(_geom_int);
	for(int i=0; i<3; i++){

		float deltaI = (vel_error(i) + _c1 * pos_error(i)) * dt;

		if(PX4_ISFINITE(deltaI)){
			if( (_geom_int(i) <= -_sigma && deltaI < 0.0f) ||
					(_geom_int(i) >= _sigma && deltaI > 0.0f) ){
				deltaI = 0.0f;
			}

			_geom_int(i) += deltaI;
		}
	}

	Vector3f acc_pid =  -pos_error.emult(Kp) - vel_error.emult(Kd)
	-constrain(_geom_int, -_sigma,_sigma).emult(Ki);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_pid);

	Vector3f g_b =  _R_yaw.transpose()*Vector3f(0.0f,0.0f,CONSTANTS_ONE_G);

	_f_b = _mass * (_acc_sp - g_b);



	_f_w = _R_yaw * Vector3f(_f_b(0),0.0f,_f_b(2));


	// Rotate to Inertial Frame
	_pos 	  = _R_yaw* _pos;
	_pos_sp   = _R_yaw* _pos_sp;
	_vel 	  = _R_yaw* _vel;
	_vel_sp   = _R_yaw* _vel_sp;
	_acc_sp   = _R_yaw* _acc_sp;



}



void PositionControl::XYThrusterAttitude(vehicle_attitude_setpoint_s &att_sp)
{

	// Same for all modes
	Vector3f body_z = -_f_w/_f_w.norm();

	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	// Remains fixed
	// desired body_x and body_y axis
	Vector3f body_x = Vector3f(cos(_yaw_sp), sin(_yaw_sp), 0.0f);
	body_x.normalize();
	Vector3f body_y = Vector3f(-sinf(_yaw_sp), cosf(_yaw_sp), 0.0f);
	body_y.normalize();

	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();


	// Changes based on the current mode
	_normalization(_f_b);

	Vector3f(_f_b(0),_f_b(1),_f_b(2)).copyTo(att_sp.thrust_body);
	att_sp.yaw_sp_move_rate = _yawspeed_sp;

}

void PositionControl::XThrusterAttitude(vehicle_attitude_setpoint_s &att_sp)
{

	// Same for all modes
	Vector3f body_z = -_f_w/_f_w.norm();
	// body_z.print();


	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	// // Remains fixed
	// Vector3f body_xc = Vector3f(cos(_yaw_sp), sin(_yaw_sp), 0.0f);
	// Vector3f body_x = (body_xc - (body_xc.dot(body_z) * body_z)).normalized();

	// // desired body_y axis
	// Vector3f body_y = body_z % body_x;
	// body_y.normalize();

	Vector3f body_x = Vector3f(cos(_yaw_sp), sin(_yaw_sp), 0.0f);


	// desired body_y axis
	Vector3f body_y = body_z % body_x;
	body_y.normalize();

	// Vector3f body_x = {0.0f,0.0f,0.0f};
	// body_x = body_y % body_z;
	// body_x.normalize();



	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();


	// Changes based on the current mode
	// float thr_yz= _f_w.length();

	float thr_yz= _f_w.dot(_R.col(2));
	Vector3f thr_cmd = {_f_b(0),0.0f,thr_yz};

	// int sign =  (_f_w(2) >= 0)?  1: -1;
	// Vector3f thr_cmd = {_f_b(0),0.0f,sign*thr_yz};

	// _normalization(_f_b);
	_normalization(thr_cmd);

	//thr_cmd.print();

	// Vector3f(_f_b(0), 0.0f, _f_b(2)).copyTo(att_sp.thrust_body);
	Vector3f(thr_cmd(0), 0.0f, thr_cmd(2)).copyTo(att_sp.thrust_body);

	att_sp.yaw_sp_move_rate = _yawspeed_sp;

}


void PositionControl::YThrusterAttitude(vehicle_attitude_setpoint_s &att_sp)
{

	// Same for all modes
	Vector3f body_z = -_f_w/_f_w.norm();


	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	// Remains fixed
	Vector3f body_y = Vector3f(-sinf(_yaw_sp), cosf(_yaw_sp), 0.0f);
	body_y.normalize();


	// desired body_y axis
	Vector3f body_x = body_y % body_z;
	body_x.normalize();

	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();


	float thr_z= _f_w.length();//_f_w.dot(_R.col(2));
	int sign =  (_f_w(2) >= 0)?  1: -1;

	Vector3f thr_cmd = {0.0f,0.0f,sign*thr_z};
	//_normalization(_f_b);
	_normalization(thr_cmd);


	// Changes based on the current mode
	_normalization(_f_b);
	// _f_b.print();

	Vector3f(0.0f, _f_b(1), _f_b(2)).copyTo(att_sp.thrust_body);
	att_sp.yaw_sp_move_rate = _yawspeed_sp;

}


void PositionControl::getGeometricAttitudeSetpoint(vehicle_attitude_setpoint_s &att_sp)
{
	switch (_auto_mode)
	{
		case 1:
			XYThrusterAttitude(att_sp);
			break;

		case 2:
			XThrusterAttitude(att_sp);
			break;

		case 3:
			YThrusterAttitude(att_sp);
			break;
		case 4:
			GeometricAttitude(att_sp);
			break;


		default: //Altitude is calculated from the desired thrust direction
			GeometricAttitude(att_sp);
	}

}


void PositionControl::GeometricAttitude(vehicle_attitude_setpoint_s &att_sp)
{

	Vector3f body_z = -_f_w/_f_w.norm();

	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	Vector3f proj_body_x = Vector3f(cos(_yaw_sp), sin(_yaw_sp), 0.0f);


	// desired body_y axis
	Vector3f body_y = body_z % proj_body_x;
	body_y.normalize();

	Vector3f body_x = {0.0f,0.0f,0.0f};
	body_x = body_y % body_z;
	body_x.normalize();


	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();

	// use for the 3d thrust

	// float thr_z= _f_w.length();//_f_w.dot(_R.col(2));
	float thr_z= _f_w.dot(_R.col(2));
	Vector3f thr_cmd = {0.0f,0.0f,thr_z};


	// int sign =  (_f_w(2) >= 0)?  1: -1;
	// Vector3f thr_cmd = {0.0f,0.0f,sign*thr_z};

	//_normalization(_f_b);
	_normalization(thr_cmd);


	// Vector3f(0.0f, 0.0f, _f_b(2)).copyTo(att_sp.thrust_body);
	Vector3f(0.0f, 0.0f, thr_cmd(2)).copyTo(att_sp.thrust_body);



	att_sp.yaw_sp_move_rate = _yawspeed_sp;

}

void PositionControl::getThrustVectoringSetpoint(geometric_setpoint_s &thrust_vectoring_setpoint)const
{
	// Force setpoint in the Inertial frame
	thrust_vectoring_setpoint.thrust[0] = _thrust_sp(0);
	thrust_vectoring_setpoint.thrust[1] = _thrust_sp(1);
	thrust_vectoring_setpoint.thrust[2] = _thrust_sp(2);

	// Rotation matrix, could also send quaternions...
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
		thrust_vectoring_setpoint.rd[3*i+j] = _Rd(i, j);
		}
	}

	//Angular velocity setpoint
	thrust_vectoring_setpoint.wd[0] = _Wd(0);
	thrust_vectoring_setpoint.wd[1] = _Wd(1);
	thrust_vectoring_setpoint.wd[2] = _Wd(2);

	// Angular Acceleration setpoint
	thrust_vectoring_setpoint.wd_dot[0] = _Wd_dot(0);
	thrust_vectoring_setpoint.wd_dot[1] = _Wd_dot(1);
	thrust_vectoring_setpoint.wd_dot[2] = _Wd_dot(2);

}

void PositionControl::_normalization(matrix::Vector3f &thrust_sp)
{


	thrust_sp(0) = PX4_ISFINITE(thrust_sp(0))? thrust_sp(0) / _max_thrust_x : 0.0f;
	thrust_sp(1) = PX4_ISFINITE(thrust_sp(1)) ? thrust_sp(1) / _max_thrust_y : 0.0f;
	thrust_sp(2) = PX4_ISFINITE(thrust_sp(2)) ? thrust_sp(2) / _max_thrust_z : 0.0f;

	thrust_sp(0) = math::constrain(thrust_sp(0), -1.0f, 1.0f);
	thrust_sp(1) = math::constrain(thrust_sp(1), -1.0f, 1.0f);
	thrust_sp(2) = math::constrain(thrust_sp(2), -1.0f, 1.0f);
}



//// CUSTOM PARAMETERS FOR PLANAR FLIGHT MODE////
void PositionControl::_planar_positionControl(const float dt, const float yaw_sp)
{

	Vector3f pos_error = (_pos_sp - _pos);
	Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p) + _pos_int - _vel.emult(_gain_planar_pos_d);

	// Update integral part of velocity control
	//separate based on each individual velocity component
	_pos_int =_pos_int + pos_error.emult(_gain_planar_pos_i) * dt;

	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	// Update integral part of velocity control
	//separate based on each individual velocity component
	_pos_int = pos_error.emult(_gain_planar_pos_i) * dt;




	Vector3f vel_sp_xy=_R.transpose() * Vector3f{_vel_sp(0),_vel_sp(1),0};
	vel_sp_xy(0) = math::constrain(vel_sp_xy(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	vel_sp_xy(1) = math::constrain(vel_sp_xy(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	vel_sp_xy=_R*Vector3f{vel_sp_xy(0),vel_sp_xy(1),0};
	_vel_sp.xy()=vel_sp_xy.xy();
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}



void PositionControl::_planar_velocityControl(const float dt,const float yaw_sp)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	//gains are the same as the ones used in the tilting mode, this should be adjusted by the user
	//The parametes should be gain_vel_p and gain_vel_d
	Vector3f acc_sp_velocity = vel_error.emult(_gain_planar_vel_p) + _vel_int - _vel_dot.emult(_gain_planar_vel_d);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_planar_accelerationControl(yaw_sp);
	//Vertical acceleration
	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	const float thrust_max_squared = math::sq(_lim_thr_max);

	//Vertical thrust
	const float thrust_z_max_squared = thrust_max_squared;// - math::sq(allocated_horizontal_thrust);
	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));


	//////Compare the merit of using an anti windup
	// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	// Integrator anti-windup in vertical direction

	//Rotate the thrust
	Vector3f thr_sp_xy=_R.transpose() * Vector3f{_thr_sp(0),_thr_sp(1),0};
	Vector3f vel_xy_error=_R.transpose() * Vector3f{vel_error(0),vel_error(1),0};
	//separate the thrust for each sign

	if(thr_sp_xy(0)>=0.0f)
	{
		if ((thr_sp_xy(0) >= _lim_planar_thr_max && vel_xy_error(0) >= 0.0f) ||
		(thr_sp_xy(0)<= _lim_planar_thr_min && vel_xy_error(0) <= 0.0f)) {
		vel_xy_error(0) = 0.f;
		}
	}

	else {
		if ((thr_sp_xy(0) <= -_lim_planar_thr_max && vel_xy_error(0) <= 0.0f) ||
		(thr_sp_xy(0)>= -_lim_planar_thr_min && vel_xy_error(0) >= 0.0f)) {
		vel_xy_error(0) = 0.f;
		}

	}

	if(thr_sp_xy(1)>=0.0f)
	{
		if ((thr_sp_xy(1) >= _lim_planar_thr_max && vel_xy_error(1) >= 0.0f) ||
		(thr_sp_xy(1)<= _lim_planar_thr_min && vel_xy_error(1) <= 0.0f)) {
		vel_xy_error(1) = 0.f;
		}
	}

	else {
		if ((thr_sp_xy(1) <= -_lim_planar_thr_max && vel_xy_error(1) <= 0.0f) ||
		(thr_sp_xy(1)>= -_lim_planar_thr_min && vel_xy_error(1) >= 0.0f)) {
		vel_xy_error(1) = 0.f;
		}

	}

	thr_sp_xy(0)=thr_sp_xy(0)>=0.0f? math::min(thr_sp_xy(0),_lim_planar_thr_max): math::max(thr_sp_xy(0),-_lim_planar_thr_max);
	thr_sp_xy(1)=thr_sp_xy(1)>=0.0f? math::min(thr_sp_xy(1),_lim_planar_thr_max): math::max(thr_sp_xy(1),-_lim_planar_thr_max);


	thr_sp_xy=_R*Vector3f{thr_sp_xy(0),thr_sp_xy(1),0};
	vel_xy_error=_R*Vector3f{vel_xy_error(0),vel_xy_error(1),0};
	_thr_sp.xy()=thr_sp_xy.xy();
	vel_error.xy()=vel_xy_error.xy();


	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	//separate based on each individual velocity component
	_vel_int += vel_error.emult(_gain_planar_vel_i) * dt;


	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));


}

void PositionControl::_planar_accelerationControl(const float yaw_sp)
{

	//divide by acceleration
	Vector3f body_z = Vector3f(0, 0, CONSTANTS_ONE_G).normalized();
	Vector3f thrz;
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	float x_thrust= _acc_sp(0)*(_hover_thrust);// use a different value perhaps to scale XY since the hover value changes
	float y_thrust= _acc_sp(1)*(_hover_thrust);//similar to the weight of the uav
	//independent of each other, no need to normalize
	Vector3f bodyxy= Vector3f(x_thrust, y_thrust, 0.0);// normalized the xy vector

	thrz= body_z * collective_thrust;

	// // Project thrust to planned body attitude
	_thr_sp(0) = bodyxy(0);
	_thr_sp(1) = bodyxy(1);
	_thr_sp(2) =thrz(2);

}
//// SINGLE PLANAR PITCH CONTROL PID END ////

void PositionControl::_planar_X_positionControl(const float dt,const float yaw_sp)
{
	//could be calculated based on the current angle (tilt_angle)
	//Based on this the system could determine when to tilt and when planar motion is accessible
	//rotation_matrix(tilted-angle) * thrust_direction, check the planar locations -> @rjros
	//position error
	//check Velocity setpoint direction
	//assume gains are for this mode only, although they could be based on the direction

	//Rotate the setpoints and references to the body frame
	_pos =_R.transpose() *  _pos;
	_pos_sp =_R.transpose() * _pos_sp;
	_vel = _R.transpose()*_vel;
	_vel_sp = _R.transpose()*_vel_sp;
	_acc_sp = _R.transpose()*_acc_sp;
	_vel_int = _R.transpose()*_vel_int;


	Vector3f _position_gains = {_gain_planar_pos_p(0),_gain_pos_p(1),_gain_pos_p(2)};
	Vector3f pos_error = _pos_sp - _pos;


	Vector3f vel_sp_position = pos_error.emult(_position_gains);

	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

}


void PositionControl::_planar_X_velocityControl(const float dt, const float yaw_sp)
{

	// Rotation
	// PID velocity control
	Vector3f vel_error = (_vel_sp - _vel);
	//gains are the same as the ones used in the tilting mode, this should be adjusted by the user
	//The parametes should be gain_vel_p and gain_vel_d
	Vector3f _velocity_gains_p = {_gain_planar_vel_p(0),_gain_vel_p(1),_gain_vel_p(2)};
	Vector3f _velocity_gains_i = {_gain_planar_vel_i(0),_gain_vel_i(1),_gain_vel_i(2)};
	Vector3f _velocity_gains_d = {_gain_planar_vel_d(0),_gain_vel_d(1),_gain_vel_d(2)};

	Vector3f acc_sp_velocity = vel_error.emult(_velocity_gains_p) + _vel_int - _vel_dot.emult(_velocity_gains_d);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_planar_X_accelerationControl(yaw_sp);
	//Vertical acceleration
	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	//Planar and Tilted case
	//Force in the X axis of the body frame must be separated from the acceleration sp.

	// Vector3f th_body=_R.transpose()*_thr_sp;

	//////Compare the merit of using an anti windup
	// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	// Integrator anti-windup in vertical direction

	//Thrust Z check the pitch effects in the thrust
	Vector2f thrust_sp_xy(0,_thr_sp(1));
	float thrust_sp_xy_norm = thrust_sp_xy.norm();
	float thrust_max_squared = math::sq(_lim_thr_max);
	float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));
	// Determine how much horizontal thrust is left after prioritizing vertical control

	float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in Y axis (roll)
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp(1) = thrust_sp_xy(1) / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Vector3f vel_xy_error=_R.transpose() * Vector3f{vel_error(0),vel_error(1),0};
	//separate the thrust for each sign
	if(_thr_sp(0)>=0.0f)
	{
		if ((_thr_sp(0) >= _lim_planar_thr_max && vel_error(0) >= 0.0f) ||
		(_thr_sp(0)<= _lim_planar_thr_min && vel_error(0) <= 0.0f)) {
		vel_error(0) = 0.f;
		}
	}

	else {
		if ((_thr_sp(0) <= -_lim_planar_thr_max && vel_error(0) <= 0.0f) ||
		(_thr_sp(0)>= -_lim_planar_thr_min && vel_error(0) >= 0.0f)) {
		vel_error(0) = 0.f;
		}

	}

	_thr_sp(0)=math::min(_thr_sp(0),_lim_planar_thr_max);

	// vel_xy_error=_R*Vector3f{vel_xy_error(0),vel_xy_error(1),0};
	// Vector3f th_new=_R*_thr_sp;
	// _thr_sp.xy()=th_new.xy();
	// vel_error.xy()=vel_xy_error.xy();

	_thr_sp = _R*_thr_sp;

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_velocity_gains_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));

	_vel_int = _R*_vel_int;

	_pos_sp =_R  * _pos_sp;
	_vel_sp = _R *_vel_sp;
	_acc_sp = _R *_acc_sp;
	// _thr_sp.print();



}
void PositionControl::_planar_X_accelerationControl(const float yaw_sp)
{
	//Force in the X axis of the body frame must be separated from the acceleration sp.
	// Vector3f _thr_sp=Vector3f{0.0,0.0,0.0};

	//Vpid
	Vector3f body_z = Vector3f(0.0f, -_acc_sp(1), 0.0f);
	Vector3f g_body = Vector3f(0.0f, 0.0f, CONSTANTS_ONE_G);//(0.0f, -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	body_z = body_z + g_body;
	body_z.normalized();

	// ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Put the gravity in the rotation matrix
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);

	//Thrust back to rotation
	_thr_sp=body_z * collective_thrust;
	_thr_sp(0)=_acc_sp(0)*_hover_thrust/CONSTANTS_ONE_G;
}


void PositionControl::_planar_Y_positionControl(const float dt,const float yaw_sp)
{
	//could be calculated based on the current angle (tilt_angle)
	//Based on this the system could determine when to tilt and when planar motion is accessible
	//rotation_matrix(tilted-angle) * thrust_direction, check the planar locations -> @rjros
	//position error
	//check Velocity setpoint direction
	//assume gains are for this mode only, although they could be based on the direction
	// of the vel vector

	// P-position controller
	Vector3f pos_error = _pos_sp - _pos;

	Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p);// + _pos_int - _vel.emult(_gain_planar_pos_d);


	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	Vector3f vel_body_xy=_R.transpose() * Vector3f{_vel_sp(0),_vel_sp(1),0};

	//Vel in X axis
	vel_body_xy(0) = math::constrain(vel_body_xy(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	vel_body_xy(1) = math::constrain(vel_body_xy(0), -_lim_vel_horizontal, _lim_vel_horizontal);

	// //Vel X and Y
	// vel_body_xy.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);

	// _vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);



}


void PositionControl::_planar_Y_velocityControl(const float dt, const float yaw_sp)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	//gains are the same as the ones used in the tilting mode, this should be adjusted by the user
	//The parametes should be gain_vel_p and gain_vel_d
	Vector3f acc_sp_velocity = vel_error.emult(_gain_planar_vel_p) + _vel_int - _vel_dot.emult(_gain_planar_vel_d);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_planar_Y_accelerationControl(yaw_sp);
	//Vertical acceleration
	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	//Planar and Tilted case
	//Force in the X axis of the body frame must be separated from the acceleration sp.
	Vector3f th_body=_R.transpose()*_thr_sp;

	//////Compare the merit of using an anti windup
	// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	// Integrator anti-windup in vertical direction

	//Thrust Z check the pitch effects in the thrust
	Vector2f thrust_sp_xy(th_body(0),0.0f);
	float thrust_sp_xy_norm = thrust_sp_xy.norm();
	float thrust_max_squared = math::sq(_lim_thr_max);
	float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	th_body(2) = math::max(th_body(2), -sqrtf(thrust_z_max_squared));
	// Determine how much horizontal thrust is left after prioritizing vertical control

	float thrust_max_xy_squared = thrust_max_squared - math::sq(th_body(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in X axis (roll)
	if (thrust_sp_xy_norm > thrust_max_xy) {
		th_body(0) = thrust_sp_xy(0) / thrust_sp_xy_norm * thrust_max_xy;
	}

	Vector3f vel_xy_error=_R.transpose() * Vector3f{vel_error(0),vel_error(1),0};
	//separate the thrust for each sign
		if(th_body(1)>=0.0f)
	{
		if ((th_body(1) >= _lim_planar_thr_max && vel_xy_error(1) >= 0.0f) ||
		(th_body(1)<= _lim_planar_thr_min && vel_xy_error(1) <= 0.0f)) {
		vel_xy_error(1) = 0.f;
		}
	}

	else {
		if ((th_body(1) <= -_lim_planar_thr_max && vel_xy_error(1) <= 0.0f) ||
		(th_body(1)>= -_lim_planar_thr_min && vel_xy_error(1) >= 0.0f)) {
		vel_xy_error(1) = 0.f;
		}

	}
	th_body(1)=math::min(th_body(1),_lim_planar_thr_max);

	vel_xy_error=_R*Vector3f{vel_xy_error(0),vel_xy_error(1),0};
	Vector3f th_new=_R*th_body;
	_thr_sp.xy()=th_new.xy();
	vel_error.xy()=vel_xy_error.xy();

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));



}
void PositionControl::_planar_Y_accelerationControl(const float yaw_sp)
{
	//Force in the X axis of the body frame must be separated from the acceleration sp.
	Vector3f body_accel_sp=_R.transpose()*_acc_sp;
	Vector3f th_body=Vector3f{0.0,0.0,0.0};

	//YZ
	Vector3f body_z = Vector3f(-body_accel_sp(0), 0.0f, CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	float collective_thrust = body_accel_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	//Thrust back to rotation
	th_body=body_z * collective_thrust;
	th_body(1)=body_accel_sp(1)*_hover_thrust/CONSTANTS_ONE_G;
	_thr_sp=_R*th_body;

}



//// SINGLE PLANAR PITCH CONTROL PID ////
void PositionControl::_autoPlanar_positionControl(const float dt,const float yaw_sp)
{

	//create temp variables since the modes have different gains
	Vector3f vel_sp = _vel_sp;
	Vector3f pos_sp = _pos_sp;

	Vector3f pos_error = pos_sp - _pos;
	Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p);

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(vel_sp, vel_sp_position);

	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);


	// Check the sp direction in the body frame to select the mode
	//If X +, check what mode is needed {X+,X-,Y+,Y-}
	//If Y +, check what mode is needed {X+,X-,Y+,Y-}
	Vector3f vel_sp_body=_R.transpose() * vel_sp;

	int8_t sp_flags{0};
	// Set the flag bits based on the sign of vp_x and vp_y
	sp_flags |= (vel_sp_body(0) >= 0) ? 0b1000 : 0b0100; // 1 in the X bit for positive X
	sp_flags |= (vel_sp_body(1) >= 0) ? 0b0010 : 0b0001; // 1 in the Y bit for positive Y

	// PX4_INFO("Current diretion %d",flags);
	// check supported vehicle
	// X (+,-) supported  3
	// Only X + supported 2
	// only X - supported 3
	int CA_flags{0};
	// _CA_mode.print();
	// Check the supported mode
	CA_flags |= (_CA_mode(0)>0 ? 0b1000 : 0b0000); // Bit 3 for X+
	CA_flags |= (_CA_mode(1)>0 ? 0b0100 : 0b0000); // Bit 2 for X-
	CA_flags |= (_CA_mode(2)>0 ? 0b0010 : 0b0000); // Bit 1 for Y+
	CA_flags |= (_CA_mode(3)>0 ? 0b0001 : 0b0000); // Bit 0 for Y-


	_control_mode = (sp_flags & CA_flags);

	if (_control_mode == 0b1010 || _control_mode == 0b0110 ||
        	_control_mode == 0b1001 || _control_mode == 0b0101)
	{
		_auto_mode=1;
		_planar_positionControl(dt,_yaw_sp);
		_planar_velocityControl(dt,_yaw_sp);

	}
	else if (_control_mode == 0b1000 || _control_mode == 0b0100)
	{
		_auto_mode=2;
		_planar_X_positionControl(dt,yaw_sp);
		_planar_X_velocityControl(dt,yaw_sp);
	}

	else if (_control_mode == 0b0010 || _control_mode == 0b0001)
	{
		_auto_mode=3;
		_planar_Y_positionControl(dt,yaw_sp);
		_planar_Y_velocityControl(dt,yaw_sp);
	}
	else {
		_auto_mode=4;
		_positionControl();
		_velocityControl(dt);
	}

}


bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}


void PositionControl::getAttitudeSetpoint(const matrix::Quatf &att, const int vectoring_att_mode,
					vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	// _thr_sp.print();
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, att, vectoring_att_mode,_auto_mode,attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}



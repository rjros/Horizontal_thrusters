/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessThrustVectoringMC : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessThrustVectoringMC(ModuleParams *parent);
	virtual ~ActuatorEffectivenessThrustVectoringMC() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;
	//Use for each of the allocated rotors, [fixed,tilted]
	int numMatrices() const override { return 2; }


	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
		allocation_method_out[1] = AllocationMethod::PSEUDO_INVERSE;

	}
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
		normalize[1] = true;

	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "Thrust Vectoring MC"; }

	// void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

protected:

	void updateParams() override;
	// ActuatorVector _tilt_offsets;
	ActuatorEffectivenessRotors _mc_rotors_fixed; //< UAV rotors

	ActuatorEffectivenessRotors _mc_rotors_tilted;//< Thrustvectoring device rotors
	ActuatorEffectivenessTilts _tilts;//< Servos attached to actuators
	// struct YawTiltSaturationFlags {
	// 	bool tilt_yaw_pos;
	// 	bool tilt_yaw_neg;
	// };

	// YawTiltSaturationFlags _yaw_tilt_saturation_flags{};

	// int32_t _attitude_mode_handle;
	int _first_tilt_idx{0}; ///< applies to matrix 1, thrust vectoring components
	param_t _tilting_type_handle;//< parameter with the CA_attitude_mode
	int32_t _tilting_type{0};//< 0 -> normal uav tilting, 1-> vectoring mode
	param_t _servo_count_handle; //< number of servos
	bool _rotors_fixed_added_succesfully{0};
	bool _rotors_tilted_added_succesfully{0};
	bool _tilts_added_succesfully{0};
	// float _last_tilt_control{NAN};

	//subscribe current desired servo values

};

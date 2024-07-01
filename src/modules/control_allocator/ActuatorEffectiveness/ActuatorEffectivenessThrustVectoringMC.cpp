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
/**
 * @file ActuatorEffectivenessThrustVectoringMC.hpp
 *
 * Actuator effectiveness for vector thrusting module
 *
 * @author Ricardo Rosales
 */



#include "ActuatorEffectivenessThrustVectoringMC.hpp"

using namespace matrix;

ActuatorEffectivenessThrustVectoringMC::ActuatorEffectivenessThrustVectoringMC(ModuleParams *parent)
	: ModuleParams(parent),
	_mc_rotors_fixed(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	_mc_rotors_tilted(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	_tilts(this)
	{
		updateParams();
	}

//Update parameters
void ActuatorEffectivenessThrustVectoringMC::updateParams()
{
	ModuleParams::updateParams();
	PX4_INFO("Updated params in Thrust Vectoring Effectivness Matrix");
	// _mc_rotors_fixed=new ActuatorEffectivenessRotors(this,ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards,false);
	// _mc_rotors_tilted= new ActuatorEffectivenessRotors(this,ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards,false);
}


bool
ActuatorEffectivenessThrustVectoringMC::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// //Flags for each matrix [[fixed],[tilted]]
	_rotors_fixed_added_succesfully=false;
	 _rotors_tilted_added_succesfully=false;

	// Compare with multiple allocation matrices
	//Fixed Motors Control
	configuration.selected_matrix=0; //< chooses the matrix to be used
	//_mc_rotors_fixed.enableYawByDifferentialThrust(false);
	_rotors_fixed_added_succesfully = _mc_rotors_fixed.addActuators(configuration,false);
	// PX4_INFO("Configuration Num actuators fixed  is %d ",*configuration.num_actuators);


	// //Tiltable rotors control
	configuration.selected_matrix=1;//< thrust vectoring motors
	_rotors_tilted_added_succesfully=_mc_rotors_tilted.addActuators(configuration,true);
	// // PX4_INFO("Configuration Num actuators tilted is %d ",*configuration.num_actuators);



	// PX4_INFO("Configuration Num actuators is %d ",*configuration.num_actuators);

	//Tilts, servos
	configuration.selected_matrix=1;
	_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	// _tilts.updateTorqueSign(_mc_rotors_fixed.geometry());
	_tilts_added_succesfully = _tilts.addActuators(configuration);



	return (_rotors_fixed_added_succesfully && _tilts_added_succesfully);
}

void ActuatorEffectivenessThrustVectoringMC::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{


	//matrix with the tiltable actuators
	if (matrix_index == 1){
		for(int i=0; i<_tilts.count(); i++){
		actuator_sp(i) = actuator_sp(i) <  math::radians(_tilts.config(i).min_angle) ?  math::radians(_tilts.config(i).min_angle) : actuator_sp(i);
		actuator_sp(i) = actuator_sp(i) >  math::radians(_tilts.config(i).max_angle) ? math::radians(_tilts.config(i).max_angle) : actuator_sp(i);
		// PX4_INFO("%d) tilt_sp: %f", i, (double)actuator_sp(i));
		}
	}


}


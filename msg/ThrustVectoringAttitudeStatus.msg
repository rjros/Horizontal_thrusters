uint64 timestamp		# time since system start (microseconds)

uint16 control_mode		# Determine if the control angles are sent from sim, RC or PC
uint16 manual_orientation	# Manual oriention for the thrust vector
float32 tilt_angle		# Tilt angle
float32 torque_percentage       #fine tune the torque/force distribution in the actuators

float32[3] thrust_tilted	#normalized values for 3d thrust
float32[3] torque_tited		#normalized values for 3d torque

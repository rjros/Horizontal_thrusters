uint64 timestamp		# time since system start (microseconds)

uint8 flight_mode 	# flight mode for the thrust vectoring vehicle
float32[8] tilt_angle 	# values for servo motors
float32[3] force 	#normalized values for 3d thrust
float32[3] torque 	#normalized values for 3d torque

# TOPICS thrust_vectoring_setpoint thrust_vectoring_status

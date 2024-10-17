uint64 timestamp		# time since system start (microseconds)

float32[8] servo_angle # values for servo motors
float32[3] force #normalized values for 3d thrust
float32[3] torque #normalized values for 3d torque


####  Matrices used for the geometric controller	####
float32[9] rd 			# Row-major representation of computed rotation matrix
float32[3] wd 			# Computer angular velocity
float32[3] wd_dot 		# Computer angular acceleration



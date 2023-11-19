//This is a library
#pragma once
#include "robot_config.h"
#include "pid.h"

/**
 * @brief The robot move with both motors turning in the same speed. No PID is used. Excat directions of the
 * motors do not need to be considered. The function will handle it.
 *
 * @param driveTrain The drivetrain of the robot
 * @param speed The speed you would like the motors to turn (-100 ~ 100) (unit: %)
 */
void move_pair(DriveTrain* driveTrain, float speed){
	if (driveTrain -> motor_type != EV3_MEDIUM){
		if (driveTrain -> is_inverted == true){
			setMotorSpeed(driveTrain -> left, -speed);
			setMotorSpeed(driveTrain -> right, -speed);
		} else {
			setMotorSpeed(driveTrain -> left, speed);
			setMotorSpeed(driveTrain -> right, speed);
		}
	} else if (driveTrain -> is_inverted == true){
		setMotorSpeed(driveTrain -> left, speed);
		setMotorSpeed(driveTrain -> right, -speed);
	} else {
		setMotorSpeed(driveTrain -> left, -speed);
		setMotorSpeed(driveTrain -> right, speed);
	}
}

/**
 * @brief The drivetrain of the robot turn at speed inputted by the user. No PID is used. Excat directions
 * of the motors do not need to be considered. The function will handle it.
 *
 * @param driveTrain The drivetrain of the robot
 * @param left_speed The speed of the motor on the left (-100 ~ 100) (unit: %)
 * @param right_speed The speed of the motor on the right (-100 ~ 100) (unit: %)
 */
void move_tank(DriveTrain* driveTrain, float left_speed, float right_speed){
	if (driveTrain -> motor_type != EV3_MEDIUM){
		if (driveTrain -> is_inverted == true){
			setMotorSpeed(driveTrain -> left, -left_speed);
			setMotorSpeed(driveTrain -> right, -right_speed);
		} else {
			setMotorSpeed(driveTrain -> left, left_speed);
			setMotorSpeed(driveTrain -> right, right_speed);
		}
	} else if (driveTrain -> is_inverted == true){
		setMotorSpeed(driveTrain -> left, left_speed);
		setMotorSpeed(driveTrain -> right, -right_speed);
	} else{
		setMotorSpeed(driveTrain -> left, -left_speed);
		setMotorSpeed(driveTrain -> right, right_speed);
	}
}

/**
 * @brief Stop the robot
 *
 * @param driveTrain The drivetrain of the robot
 */
void stop_pair(DriveTrain* driveTrain){
	setMotorSpeed(driveTrain -> left, 0);
	setMotorSpeed(driveTrain -> right, 0);
}

/**
 * @brief // Get the average encoder reading of the driving motors
 *
 * @param driveTrain The drivetrain of the robot
 * @return float
 */
float get_average_encoder(DriveTrain* driveTrain){
	return (abs(getMotorEncoder(driveTrain -> left)) + abs(getMotorEncoder(driveTrain -> right))) / 2;
}

/**
 * @brief Reset both motors of the drivetrain of the robot
 *
 * @param driveTrain The drivetrain of the robot
 */
void reset_drivetrain_encoder(DriveTrain* driveTrain){
	resetMotorEncoder(driveTrain -> left);
	resetMotorEncoder(driveTrain -> right);
}

/**
 * @brief Use one of the motors from the drivetrain for turning of the robot. No PID is used.
 *
 * @param driveTrain The drivetrain of the robot
 * @param degree The degree of turning of the whole robot (unit: degree)
 * @param turning_motor The motor use for turning (LEFT/RIGHT)
 * @param speed Speed of the turning motor (-100 ~ 100) (unit: %)
 */
void single_self_turn(DriveTrain* driveTrain, float degree, direction turning_motor, float speed){
	float setpoint = (WHEEL_SEPARATION * degree) / WHEEL_RADIUS;
	if (turning_motor == LEFT){
		setMotorSpeed(driveTrain -> right, 0);
		if (driveTrain -> motor_type != EV3_MEDIUM){
			if (driveTrain -> is_inverted == true){
				moveMotorTarget(driveTrain -> left, setpoint, -speed);
			} else {
				moveMotorTarget(driveTrain -> left, setpoint, speed);
			}
		} else if (driveTrain -> is_inverted == true){
			moveMotorTarget(driveTrain -> left, setpoint, speed);
		} else{
			moveMotorTarget(driveTrain -> left, setpoint, -speed);
		}
		waitUntilMotorStop(driveTrain -> left);
	} else {
		setMotorSpeed(driveTrain -> left, 0);
		if (driveTrain -> motor_type != EV3_MEDIUM){
			if (driveTrain -> is_inverted == true){
				moveMotorTarget(driveTrain -> right, setpoint, speed);
			} else {
				moveMotorTarget(driveTrain -> right, setpoint, -speed);
			}
		} else if (driveTrain -> is_inverted == true){
			moveMotorTarget(driveTrain -> right, setpoint, speed);
		} else{
			moveMotorTarget(driveTrain -> right, setpoint, -speed);
		}
	}
	waitUntilMotorStop(driveTrain -> right);
}

/**
 * @brief Use both motors from the drivetrain for turning of the robot. No PID is used.
 *
 * @param driveTrain The drivetrain of the robot
 * @param degree The degree of turning of the whole robot (unit: degree)
 * @param turning_motor The motor use for turning (LEFT/RIGHT)
 * @param speed Speed of the turning motor (-100 ~ 100) (unit: %)
 */
void situ_self_turn(DriveTrain* driveTrain, float degree, float speed){
	float setpoint = (WHEEL_SEPARATION * degree) / WHEEL_RADIUS;
	setpoint /= 2;
	if (driveTrain -> motor_type != EV3_MEDIUM){
		if (driveTrain -> is_inverted == true){
			moveMotorTarget(driveTrain -> left, setpoint, -speed);
			moveMotorTarget(driveTrain -> right, setpoint, speed);
		} else {
			moveMotorTarget(driveTrain -> left, setpoint, speed);
			moveMotorTarget(driveTrain -> right, setpoint, -speed);
		}
	} else if (driveTrain -> is_inverted == true){
		moveMotorTarget(driveTrain -> left, setpoint, speed);
		moveMotorTarget(driveTrain -> right, setpoint, speed);
	} else{
		moveMotorTarget(driveTrain -> left, setpoint, -speed);
		moveMotorTarget(driveTrain -> right, setpoint, -speed);
	}
	waitUntilMotorStop(driveTrain -> left);
}

/**
 * @brief Move forward to a certain degree with acceleration and deceleration. No PID is used.
 * If the the sum of acceleration and deceleration distance is larger than the total distance, the total
 * distance will become the sum of acceleration and deceleration distance. The acceleration distance and
 * deceleration distance will become the half of their original value.
 *
 * @param driveTrain The drivetrain of the robot
 * @param dist The total distance to move (> 0) (unit: degree)
 * @param accel_dist The distance for acceleration (> 0) (unit: degree)
 * @param decel_dist The distance for deceleration (> 0) (unit: degree)
 * @param max_speed Maximum speed of the drivetrain (-100 ~ 100) (unit: %)
 */
void move_forward(DriveTrain* driveTrain, float dist, float accel_dist, float decel_dist, float max_speed){
	if ((accel_dist + decel_dist) > dist){
		accel_dist = dist / 2;
		decel_dist = dist / 2;
	}
	float curr_speed = 0;
	float start_pos = abs(get_average_encoder(driveTrain));
	float acceleration = (max_speed / (accel_dist + 1)) / 1000;
	if (driveTrain -> motor_type == EV3_LARGE){
			acceleration *= EV3_LARGE_MOTOR_DPS * 0.7;
	} else if (driveTrain -> motor_type == EV3_MEDIUM){
			acceleration *= EV3_MEDIUM_MOTOR_DPS * 0.7;
	} else if (driveTrain -> motor_type == NXT_LARGE){
			acceleration *= NXT_LARGE_MOTOR_DPS * 0.7;
	}
	curr_speed += acceleration;
	while (abs(get_average_encoder(driveTrain)) - start_pos < accel_dist){
		if (abs(curr_speed) > abs(max_speed)){
			curr_speed = max_speed;
		}
		move_pair(driveTrain, curr_speed);
		curr_speed += acceleration;
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(12, "%f", getMotorEncoder(motorB));
	curr_speed = max_speed;
	while (abs(get_average_encoder(driveTrain)) - start_pos < (dist - decel_dist)){
		move_pair(driveTrain, curr_speed);
	}
	//playTone(1100, 15);
	//displayTextLine(13, "%f", getMotorEncoder(motorB));
	acceleration = ((0 - abs(max_speed)) / (decel_dist + 1)) / 1000;
	if (driveTrain -> motor_type == EV3_LARGE){
			acceleration *= EV3_LARGE_MOTOR_DPS;
	} else if (driveTrain -> motor_type == EV3_MEDIUM){
			acceleration *= EV3_MEDIUM_MOTOR_DPS;
	} else if (driveTrain -> motor_type == NXT_LARGE){
			acceleration *= NXT_LARGE_MOTOR_DPS;
	}
	while (abs(get_average_encoder(driveTrain)) - start_pos < dist){
		if (abs(curr_speed) <= 10){
			while (abs(get_average_encoder(driveTrain)) - start_pos <= dist){
				curr_speed = 10;
				move_pair(driveTrain, curr_speed);
			}
		} else{
			move_pair(driveTrain, curr_speed);
			curr_speed += acceleration;
		}
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(14, "%f", getMotorEncoder(motorB));
	stop_pair(driveTrain);
}

/**
 * @brief Move backward to a certain degree with acceleration and deceleration. No PID is used.
 * If the the sum of acceleration and deceleration distance is larger than the total distance, the total
 * distance will become the sum of acceleration and deceleration distance. The acceleration distance and
 * deceleration distance will become the half of their original value.
 *
 * @param driveTrain The drivetrain of the robot
 * @param dist The total distance to move (> 0) (unit: degree)
 * @param accel_dist The distance for acceleration (> 0) (unit: degree)
 * @param decel_dist The distance for deceleration (> 0) (unit: degree)
 * @param max_speed Maximum speed of the drivetrain (-100 ~ 100) (unit: %)
 */
void move_backward(DriveTrain* driveTrain, float dist, float accel_dist, float decel_dist, float max_speed){
	if ((accel_dist + decel_dist) > dist){
		accel_dist = dist / 2;
		decel_dist = dist / 2;
	}
	float curr_speed = 0;
	float start_pos = abs(getMotorEncoder(driveTrain -> right));
	float acceleration = -(max_speed / (accel_dist + 1)) / 1000;
	if (driveTrain -> motor_type == EV3_LARGE){
			acceleration *= EV3_LARGE_MOTOR_DPS * 0.7;
	} else if (driveTrain -> motor_type == EV3_MEDIUM){
			acceleration *= EV3_MEDIUM_MOTOR_DPS * 0.7;
	} else if (driveTrain -> motor_type == NXT_LARGE){
			acceleration *= NXT_LARGE_MOTOR_DPS * 0.7;
	}
	curr_speed += acceleration;
	while (abs(getMotorEncoder(driveTrain -> right)) - start_pos < accel_dist){
		if (abs(curr_speed) > abs(max_speed)){
			curr_speed = -max_speed;
		}
		move_pair(driveTrain, curr_speed);
		curr_speed += acceleration;
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(12, "%f", getMotorEncoder(motorB));
	curr_speed = -max_speed;
	while (abs(getMotorEncoder(driveTrain -> right)) - start_pos < (dist - decel_dist)){
		move_pair(driveTrain, curr_speed);
	}
	//playTone(1100, 15);
	//displayTextLine(13, "%f", getMotorEncoder(motorB));
	acceleration = -((0 - abs(max_speed)) / (decel_dist + 1)) / 1000;
	if (driveTrain -> motor_type == EV3_LARGE){
			acceleration *= EV3_LARGE_MOTOR_DPS;
	} else if (driveTrain -> motor_type == EV3_MEDIUM){
			acceleration *= EV3_MEDIUM_MOTOR_DPS;
	} else if (driveTrain -> motor_type == NXT_LARGE){
			acceleration *= NXT_LARGE_MOTOR_DPS;
	}
	while (abs(getMotorEncoder(driveTrain -> right)) - start_pos <= dist){
		if (abs(curr_speed) <= 10){
			while (abs(getMotorEncoder(driveTrain -> right)) - start_pos <= dist){
				curr_speed = -10;
				move_pair(driveTrain, curr_speed);
			}
		} else{
			move_pair(driveTrain, curr_speed);
			curr_speed += acceleration;
		}
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(14, "%f", getMotorEncoder(motorB));
	stop_pair(driveTrain);
}
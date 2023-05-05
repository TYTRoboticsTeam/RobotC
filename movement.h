#pragma once
#include "robot_config.h"
#include "pid.h"


//Need to implement:

void move_pair(DriveTrain* driveTrain, float speed){
	if (driveTrain -> motor_type != EV3_MEDIUM){
		if (driveTrain -> is_inverted == true){
			setMotorSpeed(driveTrain -> left, -speed);
			setMotorSpeed(driveTrain -> right, -speed);
		} else {
			setMotorSpeed(driveTrain -> left, speed);
			setMotorSpeed(driveTrain -> right, speed);
		}
	} else {
		setMotorSpeed(driveTrain -> left, -speed);
		setMotorSpeed(driveTrain -> right, speed);
	}
}

void move_tank(DriveTrain* driveTrain, float left_speed, float right_speed){
	if (driveTrain -> motor_type != EV3_MEDIUM){
		if (driveTrain -> is_inverted == true){
			setMotorSpeed(driveTrain -> left, -left_speed);
			setMotorSpeed(driveTrain -> right, -right_speed);
		} else {
			setMotorSpeed(driveTrain -> left, left_speed);
			setMotorSpeed(driveTrain -> right, right_speed);
		}
	} else {
		setMotorSpeed(driveTrain -> left, -left_speed);
		setMotorSpeed(driveTrain -> right, right_speed);
	}
}

void stop_pair(DriveTrain* driveTrain){
	setMotorSpeed(driveTrain -> left, 0);
	setMotorSpeed(driveTrain -> right, 0);
}

// Get the average encoder reading of the driving motors
float get_average_encoder(DriveTrain* driveTrain){
	if (driveTrain -> motor_type != EV3_MEDIUM){
		return abs(getMotorEncoder(driveTrain -> left) + getMotorEncoder(driveTrain -> right)) / 2;
	} else{
		return abs(getMotorEncoder(driveTrain -> left) - getMotorEncoder(driveTrain -> right)) / 2;
	}
}

void reset_encoder(DriveTrain* driveTrain){
	resetMotorEncoder(driveTrain -> left);
	resetMotorEncoder(driveTrain -> right);
}

// Move forward straightly to a certain degree with acceleration applied
// It may stop other thread when running
// Set accel_dist/decel_dist to 0 if you do not want acceleration/deceleration
// dist(degree), accel_dist(degree), decel_dist(degree), max_speed(%)
void move_forward(DriveTrain* driveTrain, float dist, float accel_dist, float decel_dist, float max_speed){
	if ((accel_dist + decel_dist) > dist){
		accel_dist = dist / 2;
		decel_dist = dist / 2;
	}
	float curr_speed = 0;
	float start_pos = abs(getMotorEncoder(driveTrain -> right));
	float acceleration = (max_speed / (accel_dist + 1)) / 1000;
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
			curr_speed = max_speed;
		}
		move_pair(driveTrain, curr_speed);
		curr_speed += acceleration;
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(12, "%f", getMotorEncoder(motorB));
	curr_speed = max_speed;
	while (abs(getMotorEncoder(driveTrain -> right)) - start_pos < (dist - decel_dist)){
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
	while (abs(getMotorEncoder(driveTrain -> right)) - start_pos < dist){
		if (abs(curr_speed) <= 10){
			while (abs(getMotorEncoder(driveTrain -> right)) - start_pos <= dist){
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

// Move backward straightly to a certain degree with acceleration applied
// It may stop other thread when running
// Set accel_dist/decel_dist to 0 if you do not want acceleration/deceleration
// dist(degree), accel_dist(degree), decel_dist(degree), max_speed(%(positive))
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

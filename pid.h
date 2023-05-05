#pragma once
#include "robot_config.h"
#include "movement.h"
#include "wheelbase.h"

// Turn to a specific degree using the gyro
// Set need_reset to true if you want to reset the gyro before turning
// setpoint(degree), speed(%), time(millisecond)
void gyro_selfturn(tSensors gyro, DriveTrain* driveTrain, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, float time){
	static bool init = true;
	static float error;
	static float integral;
	static float PID;
	clearTimer(T1);
	if (init){
		if (need_reset) resetGyro(gyro);
		error = 0;
		integral = 0;
		PID = 0;
		clearTimer(T1);
		init = false;
	}
	if ((getGyroDegrees(gyro) == setpoint && getGyroRate(gyro) == 0) || time1[T1] >= time){
		init = true;
		stop_pair(driveTrain);
	} else{
		error = setpoint - getGyroDegrees(gyro);
		integral += error;
		PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
		PID *= speed;
		move_tank(driveTrain, PID, -PID);
	}
}

// Move straight line using gyro PID
// Set need_reset to true if you want to reset the gyro before turning
// setpoint is the degree you want the robot to follow
// setpoint(degree), speed(%)
void gyro_straightForward(tSensors gyro, DriveTrain* driveTrain, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, bool condition){
	static bool init = true;
	static float start_pos;
	static float error;
	static float integral;
	static float PID;
	if (init){
		if (need_reset) resetGyro(gyro);
		start_pos = abs(getMotorEncoder(driveTrain -> right));
		error = 0;
		integral = 0;
		PID = 0;
		init = false;
	}
	if (condition){
		init = true;
		stop_pair(driveTrain);
	} else {
		error = setpoint - getGyroDegrees(gyro);
		integral += error;
		PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
		move_tank(driveTrain, speed + PID, speed - PID);
	}
}

// Move straight line backward using gyro PID
// Set need_reset to true if you want to reset the gyro before turning
// setpoint is the degree you want the robot to follow
// setpoint(degree), speed(%)
void gyro_straightBackward(tSensors gyro, DriveTrain* driveTrain, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, bool condition){
	static bool init = true;
	static float start_pos;
	static float error;
	static float integral;
	static float PID;
	if (init){
		if (need_reset) resetGyro(gyro);
		start_pos = abs(getMotorEncoder(driveTrain -> right));
		error = 0;
		integral = 0;
		PID = 0;
		init = false;
	}
	if (condition){
		init = true;
		stop_pair(driveTrain);
	} else {
		error = setpoint - getGyroDegrees(gyro);
		integral += error;
		PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
		move_tank(driveTrain, -speed + PID, -speed - PID);
	}
}

// Line Track using one two colour sensors
// Set the condition you want to stop the function
// speed(%)
void doubleColour_LineTrackPID(tSensors CS_left, tSensors CS_right, DriveTrain* driveTrain, float kp, float ki, float kd, float speed, bool condition){
	static bool init = true;
	static float error;
	static float integral;
	static float last_error;
	static float PID;
	if (init){
		error = 0;
		integral = 0;
		last_error = 0;
		PID = 0;
		init = false;
	}
	if (condition){
		init = true;
		stop_pair(driveTrain);
	} else {
		error = getColorReflected(CS_left) - getColorReflected(CS_right);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		move_tank(driveTrain, speed + PID, speed - PID);
		last_error = error;
	}
}

//This is a library
#pragma once
#include "robot_config.h"
#include "movement.h"
#include "wheelbase.h"
#include "display.h"
#include "calculation.h"

/**
 * @brief Turn to a specific degree for a single motor using PID
 *
 * @param mot The motor to turn
 * @param need_reset Set true to reset the motor encoder reading before turning (true/false)
 * @param setpoint The degree the motor will stop at (any degree) (unit: degree)
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motor while turning (>0) (unit: %)
 * @param timer Timer to be use (T1 ~ T4)
 * @param time The time limit for outlooping (unit: millisecond)
 * @return bool
 */
bool monoMotor_degreePID(tMotor mot, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, TTimers timer, float time){
	static bool init = true;
	static float error;
	static float integral;
	static float PID;
	static float last_error;
	if (init){
		if (need_reset) resetMotorEncoder(mot);
		error = 0;
		integral = 0;
		PID = 0;
		last_error = 0;
		clearTimer(timer);
		init = false;
	}
	if ((getMotorEncoder(mot) == setpoint && getMotorRPM(mot) == 0) || time1[timer] >= time){
		init = true;
		setMotorSpeed(mot, 0);
		return true;
	} else {
		error = setpoint - getMotorEncoder(mot);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		PID *= speed;
		setMotorSpeed(mot, PID);
		last_error = error;
		return false;
	}
}

/**
 * @brief Turn to a specific degree without using Gyro. PID is used.
 *
 * @param driveTrain The drivetrain of the robot
 * @param degree The degree of turning of the whole robot (unit: degree)
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param timer Timer to be use (T1 ~ T4)
 * @param time The time limit for outlooping (unit: millisecond)
 * @return bool
 */
bool situ_self_turn_PID(DriveTrain* driveTrain, float degree, float kp, float ki, float kd, float speed, TTimers timer, float time){
	static bool init = true;
	static float setpoint;
	static float error_left;
	static float error_right;
	static float integral_left;
	static float integral_right;
	static float last_error_left;
	static float last_error_right;
	static float PID_left;
	static float PID_right;
	if (init){
		setpoint = (WHEEL_SEPARATION * degree) / (WHEEL_RADIUS * 2);
		error_left = 0;
		error_right = 0;
		integral_left = 0;
		integral_right = 0;
		last_error_left = 0;
		last_error_right = 0;
		PID_left = 0;
		PID_right = 0;
		clearTimer(timer);
		init = false;
	}
	if ((abs(getMotorEncoder(driveTrain -> left)) == setpoint && getMotorRPM(driveTrain -> left) == 0 && abs(getMotorEncoder(driveTrain -> right)) == setpoint && getMotorRPM(driveTrain -> right) == 0) || time1[timer] >= time){
		init = true;
		stop_pair(driveTrain);
		return true;
	} else{
		error_left = setpoint - abs(getMotorEncoder(driveTrain -> left));
		integral_left += error_left;
		PID_left = error_left * kp + integral_left * ki + (error_left - last_error_left) * kd;
		PID_left *= speed;
		error_right = setpoint - abs(getMotorEncoder(driveTrain -> right));
		integral_right += error_right;
		PID_right = error_right * kp + integral_right * ki + (error_right - last_error_right) * kd;
		PID_right *= speed;
		move_tank(driveTrain, PID_left, -PID_right);
		last_error_left = error_left;
		last_error_right = error_right;
		return false;
	}
}

/**
 * @brief Turn to a specific degree without using the Gyro with single motor only. PID is used.
 *
 * @param driveTrain The drivetrain of the robot
 * @param dir The motor to do the turning (LEFT/RIGHT)
 * @param degree The degree of turning of the whole robot (unit: degree)
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param timer Timer to be use (T1 ~ T4)
 * @param time The time limit for outlooping (unit: millisecond)
 * @return bool
 */
bool single_self_turnPID(DriveTrain* driveTrain, direction dir, float degree, float kp, float ki, float kd, float speed, TTimers timer, float time){
	static bool init = true;
	float setpoint;
	float error_left = 0;
	float error_right = 0;
	float integral_left = 0;
	float integral_right = 0;
	float last_error_left = 0;
	float last_error_right = 0;
	float PID_left = 0;
	float PID_right = 0;
	if (init){
		setpoint = (WHEEL_SEPARATION * degree) / WHEEL_RADIUS;
		error_left = 0;
		error_right = 0;
		integral_left = 0;
		integral_right = 0;
		last_error_left = 0;
		last_error_right = 0;
		PID_left = 0;
		PID_right = 0;
		clearTimer(timer);
		init = false;
	}
	if (dir == LEFT){
		if (abs(getMotorEncoder(driveTrain -> left)) == setpoint && getMotorRPM(driveTrain -> left) == 0 || time1[timer] >= time){
			init = true;
			stop_pair(driveTrain);
			return true;
		} else {
			error_left = setpoint - abs(getMotorEncoder(driveTrain -> left));
			integral_left += error_left;
			PID_left = error_left * kp + integral_left * ki + (error_left - last_error_left) * kd;
			PID_left *= speed;
			move_tank(driveTrain, PID_left, 0);
			last_error_left = error_left;
			return false;
		}
	} else {
		if (abs(getMotorEncoder(driveTrain -> right)) == setpoint && getMotorRPM(driveTrain -> right) == 0 || time1[timer] >= time){
			init = true;
			stop_pair(driveTrain);
			return true;
		} else {
			error_right = setpoint - abs(getMotorEncoder(driveTrain -> right));
			integral_right += error_right;
			PID_right = error_right * kp + integral_right * ki + (error_right - last_error_right) * kd;
			PID_right *= speed;
			move_tank(driveTrain, 0, PID_right);
			last_error_right = error_right;
			return false;
		}
	}
}

/**
 * @brief Turn to a specific degree using the gyro. PID is used.
 *
 * @param gyro Port of the gyro (S1 ~ S4)
 * @param driveTrain The drivetrain of the robot
 * @param need_reset Reset the gyro or not. (true/false)
 * @param setpoint The degree of the gyro to be faced (unit: degree)
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param timer Timer to be use (T1 ~ T4)
 * @param time The time limit for outlooping (unit: millisecond)
 * @return bool
 */
bool gyro_selfturn(tSensors gyro, DriveTrain* driveTrain, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, TTimers timer, float time){
	static bool init = true;
	static float error;
	static float integral;
	static float PID;
	if (init){
		if (need_reset) resetGyro(gyro);
		error = 0;
		integral = 0;
		PID = 0;
		clearTimer(timer);
		init = false;
	}
	if ((getGyroDegrees(gyro) == setpoint && getGyroRate(gyro) == 0) || time1[timer] >= time){
		init = true;
		stop_pair(driveTrain);
		return true;
	} else{
		error = setpoint - getGyroDegrees(gyro);
		integral += error;
		PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
		PID *= speed;
		move_tank(driveTrain, PID, -PID);
		return false;
	}
}

// Turn to a specific degree without using the gyro with single motor only
// Tune the WHEEL_SEPARATION variable in robot_config.h if you want to use this function
// degree(degree), speed(%), time(millisecond)
/**
 * @brief Turn to a specific degree using the gyro with single motor only
 *
 * @param gyro Port of the gyro (S1 ~ S4)
 * @param driveTrain The drivetrain of the robot
 * @param dir The motor to do the turning (LEFT/RIGHT)
 * @param need_reset Reset the gyro or not. (true/false)
 * @param setpoint The degree of the gyro to be faced (unit: degree)
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param timer Timer to be use (T1 ~ T4)
 * @param time The time limit for outlooping (unit: millisecond)
 * @return bool
 */
bool single_gyro_turnPID(tSensors gyro, DriveTrain* driveTrain, direction dir, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, TTimers timer, float time){
	static bool init = true;
	static float error;
	static float integral;
	static float PID;
	if (init){
		if (need_reset) resetGyro(gyro);
		error = 0;
		integral = 0;
		PID = 0;
		clearTimer(timer);
		init = false;
	}
	if ((getGyroDegrees(gyro) == setpoint && getGyroRate(gyro) == 0) || time1[timer] >= time){
		init = true;
		stop_pair(driveTrain);
		return true;
	} else {
		if (dir == LEFT){
			error = setpoint - getGyroDegrees(gyro);
			integral += error;
			PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
			PID *= speed;
			move_tank(driveTrain, PID, 0);
			return false;
		} else {
			error = setpoint - getGyroDegrees(gyro);
			integral += error;
			PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
			PID *= speed;
			move_tank(driveTrain, 0, -PID);
			return false;
		}
	}
}

/**
 * @brief Move forward straightly using gyro. PID is used.
 *
 * @param gyro Port of the gyro (S1 ~ S4)
 * @param driveTrain The drivetrain of the robot
 * @param need_reset Reset the gyro or not. (true/false)
 * @param setpoint The degree of the gyro to be faced (unit: degree)
 * @param kp
 * @param ki
 * @param kd
  * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param condition The condition for outlooping
 * @return bool
 */
bool gyro_straightForward(tSensors gyro, DriveTrain* driveTrain, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, bool condition){
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
		return true;
	} else {
		error = setpoint - getGyroDegrees(gyro);
		integral += error;
		PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
		move_tank(driveTrain, speed + PID, speed - PID);
		return false;
	}
}

/**
 * @brief Move backward straightly using gyro. PID is used.
 *
 * @param gyro Port of the gyro (S1 ~ S4)
 * @param driveTrain The drivetrain of the robot
 * @param need_reset Reset the gyro or not. (true/false)
 * @param setpoint The degree of the gyro to be faced (unit: degree)
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param condition The condition for outlooping
 * @return bool
 */
bool gyro_straightBackward(tSensors gyro, DriveTrain* driveTrain, bool need_reset, float setpoint, float kp, float ki, float kd, float speed, bool condition){
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
		return true;
	} else {
		error = setpoint - getGyroDegrees(gyro);
		integral += error;
		PID = error * kp + integral * ki - getGyroRate(gyro) * kd;
		move_tank(driveTrain, -speed + PID, -speed - PID);
		return false;
	}
}

/**
 * @brief Move forward straightly without using gyro. PID is used.
 *
 * @param driveTrain The drivetrain of the robot
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param condition The condition for outlooping
 */
bool wb_straightForward(DriveTrain* driveTrain, float kp, float ki, float kd, float speed, bool condition){
	static bool init = true;
	static float start_pos;
	static float error;
	static float integral;
	static float last_error;
	static float PID;
	if (init){
		reset_drivetrain_encoder(driveTrain);
		start_pos = abs(getMotorEncoder(driveTrain -> right));
		error = 0;
		integral = 0;
		last_error = 0;
		PID = 0;
		init = false;
	}
	if (condition){
		stop_pair(driveTrain);
		init = true;
		return true;
	} else {
		//displayTextLine(5, "%d", getMotorEncoder(motorB));
		//displayTextLine(7, "%d", getMotorEncoder(motorC));
		//displayTextLine(9, "%d", getMotorSpeed(motorB));
		//displayTextLine(11, "%d", getMotorSpeed(motorC));
		error = abs(getMotorEncoder(driveTrain -> left)) - abs(getMotorEncoder(driveTrain -> right));
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		move_tank(driveTrain, speed - PID, speed + PID);
		last_error = error;
		return false;
	}
}

/**
 * @brief Move backward straightly without using gyro. PID is used.
 *
 * @param driveTrain The drivetrain of the robot
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param condition The condition for outlooping
 */
bool wb_straightBackward(DriveTrain* driveTrain, float kp, float ki, float kd, float speed, bool condition){
	static bool init = true;
	static float start_pos;
	static float error;
	static float integral;
	static float last_error;
	static float PID;
	if (init){
		reset_drivetrain_encoder(driveTrain);
		start_pos = abs(getMotorEncoder(driveTrain -> right));
		error = 0;
		integral = 0;
		last_error = 0;
		PID = 0;
		init = false;
	}
	if (condition){
		init = true;
		stop_pair(driveTrain);
		return true;
	} else {
		//displayTextLine(5, "%d", getMotorEncoder(motorB));
		//displayTextLine(7, "%d", getMotorEncoder(motorC));
		//displayTextLine(9, "%d", getMotorSpeed(motorB));
		//displayTextLine(11, "%d", getMotorSpeed(motorC));
		error = abs(getMotorEncoder(driveTrain -> left)) - abs(getMotorEncoder(driveTrain -> right));
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		move_tank(driveTrain, -speed + PID, -speed - PID);
		last_error = error;
		return false;
	}
}

//TODO: Add deceleration
/**
 * @brief Line follow using one colour sensor only.
 *
 * @param CS Port of the colour sensor (S1 ~ S4)
 * @param at_left State that whether the using colour sensor is placed at the left of the line or not. (true/false)
 * @param driveTrain The drivetrain of the robot
 * @param setpoint The reflected light value to be pursued
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param condition The condition for outlooping
 */
bool monoColour_LineTrackPID(tSensors CS, bool at_left, DriveTrain* driveTrain, float setpoint, float kp, float ki, float kd, float speed, bool condition){
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
		return true;
	} else {
		error = setpoint - getColorReflected(CS);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		if (at_left){
			move_tank(driveTrain, speed - PID, speed + PID);
		} else {
			move_tank(driveTrain, speed + PID, speed - PID);
		}
		last_error = error;
		return false;
	}
}

/**
 * @brief Line follow using two colour sensors.
 *
 * @param CS_left Port of the left colour sensor (S1 ~ S4)
 * @param CS_right Port of the right colour sensor (S1 ~ S4)
 * @param driveTrain The drivetrain of the robot
 * @param kp
 * @param ki
 * @param kd
 * @param speed The speed of the motors to turn (>0) (unit: %)
 * @param condition The condition for outlooping
 */
bool doubleColour_LineTrackPID(tSensors CS_left, tSensors CS_right, DriveTrain* driveTrain, float kp, float ki, float kd, float speed, bool condition){
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
		return true;
	} else {
		error = getColorReflected(CS_left) - getColorReflected(CS_right);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		move_tank(driveTrain, speed + PID, speed - PID);
		last_error = error;
		return false;
	}
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
void monoColour_AD_PID(DriveTrain* driveTrain, tSensors CS, bool at_left, float dist, float accel_dist, float decel_dist, float setpoint, float kp, float ki, float kd, float max_speed){
	if ((accel_dist + decel_dist) > dist){
		accel_dist = dist / 2;
		decel_dist = dist / 2;
	}
	float error = 0;
	float integral = 0;
	float last_error = 0;
	float PID = 0;
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
		error = setpoint - getColorReflected(CS);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		if (at_left){
			move_tank(driveTrain, curr_speed - PID, curr_speed + PID);
		} else {
			move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
		}
		last_error = error;
		curr_speed += acceleration;
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(12, "%f", getMotorEncoder(motorB));
	curr_speed = max_speed;
	while (abs(get_average_encoder(driveTrain)) - start_pos < (dist - decel_dist)){
		error = setpoint - getColorReflected(CS);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		if (at_left){
			move_tank(driveTrain, curr_speed - PID, curr_speed + PID);
		} else {
			move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
		}
		last_error = error;
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
				error = setpoint - getColorReflected(CS);
				integral += error;
				PID = error * kp + integral * ki + (error - last_error) * kd;
				if (at_left){
					move_tank(driveTrain, curr_speed - PID, curr_speed + PID);
				} else {
					move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
				}
				last_error = error;
			}
		} else{
			error = setpoint - getColorReflected(CS);
			integral += error;
			PID = error * kp + integral * ki + (error - last_error) * kd;
			if (at_left){
				move_tank(driveTrain, curr_speed - PID, curr_speed + PID);
			} else {
				move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
			}
			last_error = error;
			curr_speed += acceleration;
		}
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(14, "%f", getMotorEncoder(motorB));
	stop_pair(driveTrain);
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
void doubleColour_AD_PID(DriveTrain* driveTrain, tSensors CS_left, tSensors CS_right, float dist, float accel_dist, float decel_dist, float kp, float ki, float kd, float max_speed){
	if ((accel_dist + decel_dist) > dist){
		accel_dist = dist / 2;
		decel_dist = dist / 2;
	}
	float error = 0;
	float integral = 0;
	float last_error = 0;
	float PID = 0;
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
		error = getColorReflected(CS_left) - getColorReflected(CS_right);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
		last_error = error;
		curr_speed += acceleration;
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(12, "%f", getMotorEncoder(motorB));
	curr_speed = max_speed;
	while (abs(get_average_encoder(driveTrain)) - start_pos < (dist - decel_dist)){
		error = getColorReflected(CS_left) - getColorReflected(CS_right);
		integral += error;
		PID = error * kp + integral * ki + (error - last_error) * kd;
		move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
		last_error = error;
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
				error = getColorReflected(CS_left) - getColorReflected(CS_right);
				integral += error;
				PID = error * kp + integral * ki + (error - last_error) * kd;
				move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
				last_error = error;
			}
		} else{
			error = getColorReflected(CS_left) - getColorReflected(CS_right);
			integral += error;
			PID = error * kp + integral * ki + (error - last_error) * kd;
			move_tank(driveTrain, curr_speed + PID, curr_speed - PID);
			last_error = error;
			curr_speed += acceleration;
		}
		delay(1);
	}
	//playTone(784, 15);
	//displayTextLine(14, "%f", getMotorEncoder(motorB));
	stop_pair(driveTrain);
}
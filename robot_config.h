#pragma once
//#pragma config(Sensor, S1,     CS1,            sensorEV3_Color)
//#pragma config(Sensor, S2,     CS2,            sensorEV3_Color)
//#pragma config(Motor,  motorB,          MB,            tmotorEV3_Large, PIDControl, driveLeft, encoder)
//#pragma config(Motor,  motorC,          MC,            tmotorEV3_Large, PIDControl, driveRight, encoder)

bool start_main = false;

bool buttonListener(const char button){
	bool is_bumped = false;
	if (getButtonPress(button) && is_bumped == false){
		while (getButtonPress(button) == 1){
			is_bumped = false;
		}
		is_bumped = true;
	} else {
		is_bumped = false;
	}
	return is_bumped;
}

typedef enum{
	A,
	B,
	C,
	D
} motorPort;

typedef enum{
	EV3_LARGE,
	EV3_MEDIUM,
	NXT_LARGE,
} Motor_Type;

typedef struct {
	tMotor left;
	tMotor right;
	Motor_Type motor_type;
	bool is_inverted;
} DriveTrain;

typedef enum{
	LEFT,
	RIGHT,
} direction;

#define WHEEL_RADIUS 28 //unit in mm

// To use the "turn" function well, you have to tune this value accurately
// If you are using a gyro, its ok not to bother with this variable
#define WHEEL_SEPARATION 139 //unit in mm

#define COLOUR_SENSORS_DISTANCE 27 //unit in mm

#define MOTOR_TO_CS_DISTANCE 125 //unit in mm

#define ACCEL 10 //unit in %/s

#define MAX_POWER 100 //unit in %

#define TIME_ZERO_TO_MAX MAX_SPEED / ACCEL

#define EV3_LARGE_MOTOR_RPM 105 //unit in RPM
#define EV3_LARGE_MOTOR_DPS EV3_LARGE_MOTOR_RPM * 6 //unit in degree/second

#define EV3_MEDIUM_MOTOR_RPM 250 //unit in RPM
#define EV3_MEDIUM_MOTOR_DPS EV3_MEDIUM_MOTOR_RPM * 6 //unit in degree/second

#define NXT_LARGE_MOTOR_RPM 80 //unit in RPM (Not yet tested)
#define NXT_LARGE_MOTOR_DPS NXT_LARGE_MOTOR_RPM * 6 //unit in degree/second

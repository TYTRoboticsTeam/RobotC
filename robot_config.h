#pragma once
// Initialize your robot configurations here.

bool start_main = false;

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

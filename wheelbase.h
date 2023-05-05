#pragma once
#include "wheelbase.h"
#include "robot_config.h"
#include "pid.h"
#include "movement.h"
#include "detection.h"

bool is_wb_init = false;

DriveTrain wheelbase;

bool wb_init(bool init){
	if (!init){
		//wheelbase.left = motorB;
		//wheelbase.right = motorC;
		//wheelbase.motor_type = EV3_LARGE;
		//wheelbase.is_inverted = true;
	}
	return true;
}

task wheelbase_thread(){
	if (is_wb_init != true){
		is_wb_init = wb_init(is_wb_init);
	}
	playSound(soundFastUpwardTones);
	while (true){
		if (!start_main) continue;
		// Write your code here
		delay(50);
	}
}

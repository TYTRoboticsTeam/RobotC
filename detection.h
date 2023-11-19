#pragma once
#include "robot_config.h"

/**
 * @brief Get the estimated battery percentage of the EV3 brick
 * 
 * @return float
 */
float getBatteryPercentage(){
	return (getBatteryVoltage() * 1000 - 6000) / (8400 - 6000) * 100;
}

task detection_task(){
	while (true){
		// Write your code here
		delay(50);
	}
}
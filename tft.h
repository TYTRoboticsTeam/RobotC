#pragma once
#include "robot_config.h"
#include "detection.h"

task tft_thread(){
	while (true){
		delay(100);
		displayTextLine(1, "%s: %.2f", "Battery: ", getBatteryPercentage());
		//Write your code here
	}
}

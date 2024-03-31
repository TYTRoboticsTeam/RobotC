#include "robot_config.h"
#include "wheelbase.h"
#include "movement.h"
#include "tft.h"
#include "detection.h"
#include "display.h"
#include "calculation.h"

/**
 * @brief Temporary stop the robot from moving
 */
void pause(){
	while(getButtonPress(buttonEnter) != 1){
		stop_pair(wheelbase);
	}
	delay(1000);
}

void program_one(){
	stopAllTasks();
}

void program_two(){
	stopAllTasks();
}

void program_three(){
	stopAllTasks();
}

task main(){
	if (getBatteryPercentage() < 15){
		playSound(soundException);
		delay(1000);
		stopAllTasks();
	}
	startTask(wheelbase_thread);
	startTask(tft_thread);
	startTask(detection_task);
	while (true){
		if (!start_main){
			int which_program = -1;
			while (true) {
				which_program = pre_program_selection_page();
				switch (which_program){
					case 1:
						program_one();
						break;
					case 2:
						program_two();
						break;
					case 3:
						program_three();
						break;
					defalut:
						program_one();
				}
			}
		}
	}
}

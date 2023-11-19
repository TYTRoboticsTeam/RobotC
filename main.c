#include "robot_config.h"
#include "wheelbase.h"
#include "movement.h"
#include "movement.h"
#include "tft.h"
#include "detection.h"

task main(){
	if (getBatteryPercentage() < 65){
		playSound(soundException);
		delay(1000);
		stopAllTasks();
	}
	startTask(wheelbase_thread);
	startTask(tft_thread);
	startTask(detection_task);
	while (true){
		if (!start_main){
			if (getButtonPress(buttonEnter) != 1) continue;
			else start_main = true;
		}
		//Write your code here (Altho it is not recommened to put all the code in main())
	}
}
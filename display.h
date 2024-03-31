#pragma once
#include "robot_config.h"

// Please write down program name here

#define MAX_LINE 8
#define MAX_SPACE 28
#define SPACE ' '

string instruction = "<- Back   o Enter";

void cut_text(char * text, int num_char){
	char dummy[30] = '\0';
	strncpy(dummy, text, num_char);
	dummy[num_char] = '\0';
	strcpy(text, dummy);
}

void cut_text_remain(char * text, char * remain, int num_char){
	memcpy(remain, text + num_char, strlen(text));
}

void smart_display_left(int line, char * text, bool invert_bg){
	if (strlen(text) > MAX_SPACE){
		char remain[50] = '\0';
		cut_text_remain(text, remain, MAX_SPACE);
		cut_text(text, MAX_SPACE);
		smart_display_left(line + 1, remain, invert_bg);
	}
	if (invert_bg){
		displayInverseString(line, "%s", text);
	} else {
		displayString(line, "%s", text);
	}
}

void smart_display_center(int line, char * text, bool invert_bg){
	if (strlen(text) > MAX_SPACE){
		char remain[50] = '\0';
		cut_text_remain(text, remain, MAX_SPACE);
		cut_text(text, MAX_SPACE);
		smart_display_center(line + 1, remain, invert_bg);
	}
	int num_space = ceil((MAX_SPACE - strlen(text)) / 2.0);
	char new_text[MAX_SPACE] = '\0';
	for (int i = 0; i < num_space; i++){
		new_text[i] = SPACE;
		new_text[i + 1] = '\0';
	}
	strcat(new_text, text);
	char showing_text[MAX_SPACE] = '\0';
	strcpy(showing_text, new_text);
	if (invert_bg){
		displayInverseString(line, "%s", &showing_text);
	} else {
		displayString(line, "%s", &showing_text);
	}
}

void smart_display_right(int line, char * text, bool invert_bg){
	if (strlen(text) > MAX_SPACE){
		char remain[50] = '\0';
		cut_text_remain(text, remain, MAX_SPACE);
		cut_text(text, MAX_SPACE);
		smart_display_right(line + 1, remain, invert_bg);
	}
	int num_space = MAX_SPACE - strlen(text);
	char new_text[MAX_SPACE] = '\0';
	for (int i = 0; i < num_space; i++){
		new_text[i] = SPACE;
		new_text[i + 1] = '\0';
	}
	strcat(new_text, text);
	char showing_text[MAX_SPACE] = '\0';
	strcpy(showing_text, new_text);
	if (invert_bg){
		displayInverseString(line, "%s", &showing_text);
	} else {
		displayString(line, "%s", &showing_text);
	}
}

int program_selection_page(){
	bool outloop = false;
	bool proceed = false;
	int num_programs = 3;
	int selecting = 1;
	eraseDisplay();
	while (!outloop){
		proceed = buttonListener(buttonEnter);
		outloop = proceed || buttonListener(buttonLeft);
		if (buttonListener(buttonUp) && selecting > 1){
			selecting--;
		}
		if (buttonListener(buttonDown) && selecting < num_programs){
			selecting++;
		}
		smart_display_left(1, "PROGRAM 1", selecting == 1);
		smart_display_left(3, "PROGRAM 2", selecting == 2);
		smart_display_left(5, "PROGRAM 3", selecting == 3);
		smart_display_center(15, instruction, false);
	}
	if (proceed){
		return selecting;
	} else {
		return -1;
	}
	eraseDisplay();
}

void port_view_page(){
	bool outloop = false;
	eraseDisplay();
	while (!outloop){
		outloop = buttonListener(buttonLeft);
		//Example:
		//string battery_info;
		//sprintf(battery_info, "Batt: %.1f", getBatteryPercentage());
		//smart_display_center(5, battery_info, false);
		smart_display_center(15, instruction, false);
	}
	eraseDisplay();
}

void tuning_app(float *value, float step, float upper_limit, float lower_limit, string which_item, int line){
	bool go_back = false;
	while (!go_back){
		go_back = buttonListener(buttonEnter) || buttonListener(buttonLeft);
		if (buttonListener(buttonUp) && *value < upper_limit){
			*value += step;
		}
		if (buttonListener(buttonDown) && *value > lower_limit){
			*value -= step;
		}
		string full_item = "\0";
		sprintf(full_item, "%s%f tuning", which_item, *value);
		smart_display_left(line, full_item, true);
		smart_display_center(15, instruction, false);
	}
}

void tuning_page(){
	bool outloop = false;
	bool lock_on = false;
	int num_items = 3;
	int item = 1;
	eraseDisplay();
	while (!outloop){
		lock_on = buttonListener(buttonEnter);
		outloop = buttonListener(buttonLeft);
		if (buttonListener(buttonUp) && item > 1){
			item--;
		}
		if (buttonListener(buttonDown) && item < num_items){
			item++;
		}
		//Example
		//string item_one;
		//string item_one_full;
		//sprintf(item_one, "kp: ");
		//sprintf(item_one_full, "%s%f", item_one, kp);
		//smart_display_left(1, item_one_full, item == 1);
		smart_display_center(15, instruction, false);

		if (lock_on){
			//Example
			//switch (item){
			//	case 1:
			//		tuning_app(&kp, 0.01, 999, 0, item_one, 1);
			//}
		}
	}
	eraseDisplay();
}

int pre_program_selection_page(){
	typedef enum{
		SELECT_PROGRAM = 1,
		PORT_VIEW,
		TUNING
	} pre_program_options;
	int num_options = 3;
	int option = 1;
	bool proceed = false;

	while (!proceed){
		if (buttonListener(buttonUp) && option > 1){
			option--;
		}
		if (buttonListener(buttonDown) && option < num_options){
			option++;
		}
		bool is_entered = false;
		switch (option){
			case SELECT_PROGRAM:
				is_entered = buttonListener(buttonEnter);
				if (is_entered){
					is_entered = false;
					return program_selection_page();
				}
			case PORT_VIEW:
				is_entered = buttonListener(buttonEnter);
				if (is_entered){
					is_entered = false;
					port_view_page();
				}
			case TUNING:
				is_entered = buttonListener(buttonEnter);
				if (is_entered){
					is_entered = false;
					tuning_page();
				}
		}
		smart_display_left(1, "SELECT PROGRAM", option == 1);
		smart_display_left(3, "PORT VIEW", option == 2);
		smart_display_left(5, "TUNE VALUE", option == 3);
		smart_display_center(15, instruction, false);
	}
	return true;
}

//
// Created by livello on 23.09.2021.
//

#ifndef GEAR_DRIVE_H_
#define GEAR_DRIVE_H_

#define GEAR_SAME_STATE_COUNT_FILER 20

enum ret_codes gear_falling(void);
enum ret_codes gear_rising(void);
void gear_drive_rise();
void gear_drive_fall();
void gear_drive_stop();
void timer_gear_button_event_handler(void *p_context);
void gear_init(void);
#endif GEAR_DRIVE_H_
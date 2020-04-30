#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

void set_robot(int16_t right_speed, int16_t left_speed);

//start the PI regulator thread
void pi_regulator_start(void);

int16_t get_right_speed(void);

int16_t get_left_speed(void);

uint8_t get_system_state(void);

#endif /* PI_REGULATOR_H */

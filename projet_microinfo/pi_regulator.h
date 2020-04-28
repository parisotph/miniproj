#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

void stop_robot(void);

void turn_robot(int16_t speed);

uint16_t get_measure(void);

//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */

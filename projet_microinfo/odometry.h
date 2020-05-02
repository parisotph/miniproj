/*
 * odometry.h
 *
 *  Created on: 24 avr. 2020
 *      Author: poseidon
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

/*void variation_calcul(int16_t s_r, int16_t s_l);*/

void update_robot(void);

void odometry_start(void);

uint8_t get_dist_condition(void);

uint8_t get_angle_condition(void);

uint8_t get_origin_condition(void);

float get_dist(void);

float get_teta(void);

float get_dteta(void);

int32_t get_delta(void);

void reset_odometry(void);

#endif /* ODOMETRY_H_ */

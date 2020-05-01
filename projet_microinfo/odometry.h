/*
 * odometry.h
 *
 *  Created on: 24 avr. 2020
 *      Author: poseidon
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

void variation_calcul(int16_t s_r, int16_t s_l);

void update_robot(float angle, float x_pos, float y_pos, float delta_angle, float delta_x_pos, float delta_y_pos);

void odometry_start(void);

uint8_t get_dist_condition(void);

uint8_t get_angle_condition(void);

uint8_t get_origin_condition(void);

float get_dist(void);

float get_teta(void);

void reset_odometry(void);

#endif /* ODOMETRY_H_ */

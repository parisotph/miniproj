/*
 * odometry.h
 *
 *  Created on: 24 avr. 2020
 *      Author: poseidon
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

void variation_calcul(int16_t s_r, int16_t s_l);

void reset_coordinate(float angle, float x_pos, float y_pos, float distance);

void reset_variables(uint8_t dist_reached, uint8_t angle_reached, uint8_t origin_reached, float turn_angle, float phi);

void update_robot(float angle, float x_pos, float y_pos, float delta_angle, float delta_x_pos, float delta_y_pos);

void odometry_start(void);

uint8_t get_dist_condition(void);

uint8_t get_angle_condition(void);

uint8_t get_origin_condition(void);

#endif /* ODOMETRY_H_ */

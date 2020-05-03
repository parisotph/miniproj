/*
 * odometry.h
 *
 *  Created on: 24 avr. 2020
 *      Author: poseidon
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

void variation_calcul(void);

void update_robot(void);

void odometry_start(void);

uint8_t get_dist_condition(void);

uint8_t get_angle_condition(void);

uint8_t get_origin_condition(void);

float get_dist(void);

float get_teta(void);

float get_dteta(void);

float get_xc(void);

float get_yc(void);

int32_t get_delta(void);

int32_t get_delta_r_pos(void);

int32_t get_delta_l_pos(void);

int32_t get_last_r_pos(void);

int32_t get_last_l_pos(void);

void reset_odometry(void);

#endif /* ODOMETRY_H_ */

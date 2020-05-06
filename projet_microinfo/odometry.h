/*
 * odometry.h
 *
 *  Created on: 17 avr. 2020
 *      Author: Philippe parisot and Lucas Bost
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

void reset_map(void);

void reset_odometry(void);

void odometry_start(void);

uint8_t get_dist_condition(void);

uint8_t get_angle_condition(void);

uint8_t get_origin_condition(void);

#endif /* ODOMETRY_H_ */

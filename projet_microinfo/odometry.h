/*
 * odometry.h
 *
 *  Created on: 17 avr. 2020
 *      Author: Philippe parisot and Lucas Bost
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

//Message containing informations for move.c (3 conditions)
typedef struct{
	uint8_t cd1;
	uint8_t cd2;
	uint8_t cd3;
}odm_msg_t;

void reset_map(void);

void reset_odometry(void);

void odometry_start(void);

#endif /* ODOMETRY_H_ */

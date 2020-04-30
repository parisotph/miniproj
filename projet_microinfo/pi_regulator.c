#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <VL53L0X.h>

//static uint8_t system_state = TURN;
static uint16_t mesure;
static int16_t left_speed;
static int16_t right_speed;
static uint8_t system_state = TURN;
static uint8_t first_time



void set_robot(int16_t right_speed, int16_t left_speed){
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
}


//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD || error < 0){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //uint16_t mesure;

    int16_t speed = 0;
    int16_t speed_correction;
    systime_t time;
    uint8_t n= 0;

    uint8_t first_time = 0;

    while(1){
    	time = chVTGetSystemTime();

    	mesure = VL53L0X_get_dist_mm();



    	if(system_state == TURN){
    			if(mesure < D_MAX){
    			//system_state = PURSUIT;
    				if(n >= 50){
    					right_speed = STOP;
    					left_speed = STOP;
    					set_robot(right_speed, left_speed);
    					system_state = PURSUIT;
    				}

    				else{
    					n++;
    				}

    				//stop_robot()
    			}
    			else{
    				right_speed = S_TURN;
    				left_speed = S_TURN;
    				set_robot(right_speed, left_speed);
    			}
    	}


    	if(system_state == PURSUIT){
    				if(d_reached){
    					right_speed = STOP;
    					left_speed = STOP;
    					set_robot(right_speed, left_speed);
    					system_state = COMEBACK;
    				}
    				else{
						//computes the speed to give to the motors
						//distance_cm is modified by the image processing thread
						speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
						//computes a correction factor to let the robot rotate to be in front of the line
						speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

						//if the line is nearly in front of the camera, don't rotate
						if(abs(speed_correction) < ROTATION_THRESHOLD){
							speed_correction = 0;
						}
						right_speed = speed - ROTATION_COEFF * speed_correction;
						left_speed = speed + ROTATION_COEFF * speed_correction;
						set_robot(right_speed, left_speed);
    				}
    	}


    	if (state == COMEBACK) {
    	    if(angle_reached){
    	        if(origin_reached) {
    	            right_speed = STOP;
    	            left_speed = STOP;
    	            set_robot(right_speed, left_speed);
    	            state = TURN;
    	            set_first_time(first_time);
    	        } else {
    	            right_speed = C_SPEED;
    	            left_speed = C_SPEED;
    	            set_robot(right_speed, left_speed);
    	        }
    	    } else {
    	        right_speed = S_TURN;
    	        left_speed = S_TURN;
    	        set_robot(right_speed, left_speed);
    	    }
    	}

    	//chprintf((BaseSequentialStream *)&SD3, "dist = %d<n", system_state);

    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

        //100Hz
        //chThdSleepUntilWindowed(time, time + MS2ST(10));
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

int16_t get_right_speed(void){
	return right_speed;
}

int16_t get_left_speed(void){
	return left_speed;
}

uint8_t get_system_state(void){
		return system_state;
}


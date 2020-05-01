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
#include <odometry.h>

//static uint8_t system_state = TURN;
static uint8_t target_captured;
//static uint8_t target_out_range;
static int16_t left_speed;
static int16_t right_speed;
static uint8_t system_state = TURN;



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

	if(error < CAUGHT){
		target_captured = 1;
	}

	/*if(error > OUT_RANGE){
		target_out_range = 1;
	}*/

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
    uint16_t mesure;
    uint8_t dist_reached;
    uint8_t angle_reached;
	uint8_t origin_reached;

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
    				right_speed = CST_SPEED;
    				left_speed = -CST_SPEED;
    				set_robot(right_speed, left_speed);
    			}
    	}


    	if(system_state == PURSUIT){
    				dist_reached = get_dist_condition();
    				if(dist_reached){
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


    	if(system_state == COMEBACK){
    	    angle_reached = get_angle_condition();
    		if(angle_reached){
    			origin_reached = get_origin_condition();
    	        if(origin_reached) {
    	            right_speed = STOP;
    	            left_speed = STOP;
    	            set_robot(right_speed, left_speed);
    	            reset_odometry();
    	            reset_pursuit();
    	            system_state = TURN;
    	        } else {
    	            right_speed = CST_SPEED;
    	            left_speed = CST_SPEED;
    	            set_robot(right_speed, left_speed);
    	        }
    	    } else {
    	        right_speed = CST_SPEED;
    	        left_speed = -CST_SPEED;
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

uint8_t get_target_situation(void){
	return target_captured;
}

/*uint8_t get_pursuit_situation(void){
	return target_out_range;
}*/

void reset_pursuit(void){
	target_captured = 0;
	//target_out_range = 0;
}


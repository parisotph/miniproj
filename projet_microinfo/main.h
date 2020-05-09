#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "audio/play_melody.h"
#include "audio/audio_thread.h"
#include "sensors/proximity.h"

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640                           //[pxl] size of the buffer for the image
#define WIDTH_SLOPE				5                             //[pxl] minimum width for a falling/rising edge
#define MIN_LINE_WIDTH			40                            //[pxl] minimum width for a black line
#define ROTATION_THRESHOLD		10                            //[pxl] minimum difference in position before rotation
#define ROTATION_COEFF			2                             //to convert a position deviation into speed
#define PXTOCM					1570.0f	                      //to convert line width in pixel in a distance in cm depending on lens and sensor properties
#define GOAL_DISTANCE 			10.0f                         //[cm] distance at which the robot must remain
#define MAX_DISTANCE 			25.0f                         //[cm] maximum distance between robot and target
#define ERROR_THRESHOLD			0.1f		                  //[cm] because of the noise of the camera
#define KP						800.0f                        //value found for the proportional gain
#define KI 						3.5f		                  //value for the integrator gain (must not be zero)
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)        //limit for the sum of error
#define TOF_THRESHOLD           130                           //[mm] critical distance between center of the robot and target
#define TOF_ERROR               30                            //[mm] because of the sensor instability and the line small movements
#define IR_THRESHOLD          	2000                          //critical value for proximity detection
#define STOP                    0                             //[step/s] to stop the robot
#define CST_SPEED               250                           //[step/s] all the constant speed are multiple of this speed in the project
#define PI                      3.141592654f                  //pi
#define CORRECTION				0.1f                        //[rad] correction of the angle of alignment
#define POS_FACTOR              0.0065f                       //[cm/step] factor for the distance traveled
#define TETA_FACTOR             0.002453f                     //[rad/step] factor for the robot orientation
#define PERIMETER_RADIUS        43.0f                         //[cm] radius of the demarcated zone
#define EDGE                    430                           //[mm] radius of the demarcated zone (taking into account the instability of the sensor)
#define ACHIEVE                 1                             //if a condition has been reached
#define WAIT_TIME               2500                          //[ms] time to wait between warning and possible prosecution
#define NOTE_INTENSITY          4                             //intensity of the warning sound
#define NOTE_DURATION           200                           //[ms] duration of the warning sound
#define ON                   	1                             //light is ON
#define OFF                     0                             //light is OFF
#define IRA						0                             //front right IR sensor
#define IRB						7                             //front left IR sensor
#define LIMIT                   2                             //minimum number of measurements needed per sample
#define COUNT                   6                             //number of measurements needed per sample
#define HALF_SECOND             5                             //in hundreds of milliseconds

//define the three system states
enum etat {TURN, PURSUIT, COMEBACK};

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif

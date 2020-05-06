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
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f	// experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f		// [cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f		// must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define D_MAX                   150
#define STOP                    0
#define CST_SPEED               250
#define PI                      3.141592654f
#define POS_FACTOR              0.0065f
#define TETA_FACTOR             0.002453f
#define NEAR_ORIGIN             0.5f
#define CAUGHT                  50
#define PERIMETER_RADIUS        30.0f
#define ACHIEVE                 1
#define R_FACTOR                2.65
#define WAIT_TIME               2500
#define HALF_SECOND             500
#define NOTE_INTENSITY          4
#define NOTE_DURATION           200
#define ON                   	2
#define OFF                     0
#define IRA						0
#define IRB						7
#define IR_MIN                  3000


enum etat {TURN, PURSUIT, COMEBACK};

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif

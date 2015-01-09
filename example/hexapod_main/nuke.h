/******************************************************************************
 * Inverse Kinematics for 4/6 legged bots using 3DOF lizard legs
 *
 * Auto-Generated by NUKE!
 *   http://arbotix.googlecode.com
 *
 * FRONT VIEW       ^        ==0         0==
 *     /\___/\      |       |  0==[___]==0  |
 *    /       \     Z       |               |
 *
 * TOP VIEW
 *    \       /     ^
 *     \_____/      |
 *  ___|     |___   X
 *     |_____|
 *     /     \      Y->
 *    /       \
 *****************************************************************************/

#ifndef NUKE
#define NUKE

#define LEG_COUNT   6

/* Body
 * We assume 4 legs are on the corners of a box defined by X_COXA x Y_COXA
 * Middle legs for a hexapod can be different Y, but should be halfway in X
 */
#define X_COXA      60  // MM between front and back legs /2
#define Y_COXA      60  // MM between front/back legs /2
#define M_COXA      100  // MM between two middle legs /2

/* Legs */
#define L_COXA      52  // MM distance from coxa servo to femur servo
#define L_FEMUR     82 // MM distance from femur servo to tibia servo
#define L_TIBIA     140 // MM distance from tibia servo to foot

/* Servo IDs */
#define RM_TIBIA 18
#define RF_COXA 2
#define LR_TIBIA 11
#define LF_FEMUR 3
#define RF_TIBIA 6
#define RM_FEMUR 16
#define RM_COXA 14
#define RR_COXA 8
#define LF_TIBIA 5
#define LF_COXA 1
#define LR_FEMUR 9
#define RR_FEMUR 10
#define LM_TIBIA 17
#define RF_FEMUR 4
#define LM_FEMUR 15
#define RR_TIBIA 12
#define LM_COXA 13
#define LR_COXA 7

/* A leg position request (output of body calcs, input to simple 3dof solver). */
 typedef struct{
 	int x;
 	int y;
 	int z;
 	float r;
 } ik_req_t;

/* Servo ouptut values (output of 3dof leg solver). */
 typedef struct{
 	int coxa;
 	int femur;
 	int tibia;
 } ik_sol_t;

/* Actual positions, and indices of array. */
 extern ik_req_t endpoints[LEG_COUNT];
#define RIGHT_FRONT    0
#define RIGHT_REAR     1
#define LEFT_FRONT     2
#define LEFT_REAR      3
#define RIGHT_MIDDLE   4
#define LEFT_MIDDLE    5

 extern BioloidController bioloid;

/* Parameters for manipulating body position */
extern float bodyRotX;    // body roll
extern float bodyRotY;    // body pitch
extern float bodyRotZ;    // body rotation
extern int bodyPosX;
extern int bodyPosY;

/* Parameters for gait manipulation */
extern int Xspeed;
extern int Yspeed;
extern float Rspeed;
extern int tranTime;
extern float cycleTime;
extern int stepsInCycle;
extern int liftHeight;
extern int step;

/* Gait Engine */
extern int gaitLegNo[];   // order to move legs in
extern ik_req_t gaits[];  // gait position

/* convert radians to a dynamixel servo offset */
int radToServo(float rads);
/* select a gait pattern to use */
void gaitSelect(int GaitType);

#include "gaits.h"

/* find the translation of the coxa point (x,y) in 3-space, given our rotations */
ik_req_t bodyIK(int X, int Y, int Z, int Xdisp, int Ydisp, float Zrot);
/* given our leg offset (x,y,z) from the coxa point, calculate servo values */
ik_sol_t legIK(int X, int Y, int Z);
/* ties all of the above together */
void doIK();
/* setup the starting positions of the legs. */
void setupIK();
void doIK_Z0();

#endif

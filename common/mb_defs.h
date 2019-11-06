/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR                  1 // id of left motor
#define RIGHT_MOTOR                 2 // id of right motor
#define MDIR1_CHIP                  1// chip of MDIR1 gpio pin
#define MDIR1_PIN                   28 //MDIRR1 gpio(CHIP.PIN) P9.12
#define MDIR2_CHIP                  1 // chip of MDIR2 gpio pin
#define MDIR2_PIN                   16 //  MDIRR2 gpio(CHIP.PIN) P9.15
#define MOT_BRAKE_EN            0,20  // gpio0.20  P9.41
#define MOT_1_POL                   1 // polarity of motor 1
#define MOT_2_POL                   -1 // polarity of motor 2
#define ENC_1_POL                   1 // polarity of encoder 1
#define ENC_2_POL                   -1 // polarity of encoder 2
#define MOT_1_CS                    39 // analog in of motor 1 current sense
#define MOT_2_CS                    40 // analog in of motor 2 current sense
#define GEAR_RATIO                  20.4 // gear ratio of motor
#define ENCODER_RES                 48 // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER              0.0833 // diameter of wheel in meters
#define WHEEL_BASE                  0.204 // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY       0.1 // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY      0.1 // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ            100 // main filter and control loop speed
#define DT                       0.01 // 1/sample_rate
#define PRINTF_HZ                  10 // rate of print loop
#define RC_CTL_HZ                  25 // rate of RC data update

// inner loop controller 100hz
// #define D1_GAIN			1.05
#define D1_ORDER		2
// #define D1_NUM			{-5.706, 10.18, -4.519}
// #define D1_DEN			{ 1.000, -1.695, 0.694}
#define D1_NUM_LEN		3
#define D1_DEN_LEN		3
#define D1_SATURATION_TIMEOUT	0.4

// outter loop parameters
//#define D2_GAIN			0.9
#define	D2_ORDER		1
// #define D2_NUM			{0.1583,  -0.1524}
// #define D2_DEN			{1.00000,  -0.9417}
#define D2_NUM_LEN		2
#define D2_DEN_LEN		2
#define THETA_REF_MAX		0.33

// steering controller
#define D3_KP			1.0
#define D3_KI			0.0
#define D3_KD			0.02
#define STEERING_INPUT_MAX	0.5

#define V_NOMINAL		12

// other
#define TIP_ANGLE		0.85
#define START_ANGLE		0.3
#define START_DELAY		0.4
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC		0.7

// Offset
//#define X_offset -0*3.14/180

#define DSM_DRIVE_CH 3
#define DSM_TURN_CH 4
#define DSM_TURN_POL -1
#define DSM_DRIVE_POL 1
#define DSM_DEAD_ZONE 0.04
#define DRIVE_RATE 10
#define TURN_RATE 4

// Used in odometry for Gyrodometry
#define DGO 0.09 //5 degree difference
#endif

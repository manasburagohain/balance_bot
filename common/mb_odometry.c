/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/
#include <math.h> // for M_PI

#include "../balancebot/balancebot.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float gamma){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->gamma = gamma;
}


void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    double wheelAngleR = (mb_state->d_right*2.0 * M_PI) /(ENC_2_POL * GEAR_RATIO * ENCODER_RES);
    double wheelAngleL = (mb_state->d_left*2.0 * M_PI) /(ENC_1_POL * GEAR_RATIO * ENCODER_RES);
    float d = (wheelAngleL + wheelAngleR)*(WHEEL_DIAMETER/2)/2;
    float dgamma = (wheelAngleR - wheelAngleL)*(WHEEL_DIAMETER/2)/WHEEL_BASE;
    //printf("Gamma: %f", dgamma);

    //Gyrodometry: if the difference of orientation change between gyro and encoder calculation is large, then trust gyro
    if (abs(mb_state->dgamma_gyro - dgamma) > DGO){
        dgamma = mb_state->dgamma_gyro;
    }

    mb_odometry->gamma += dgamma;
    mb_odometry->gamma = mb_clamp_radians(mb_odometry->gamma);

    mb_odometry->x  += d * cos(mb_odometry->gamma + 1/2*dgamma) ;
    mb_odometry->y += d * sin(mb_odometry->gamma + 1/2*dgamma);
}


float mb_clamp_radians(float angle){
    if(angle < -M_PI)
        angle += 2.0*M_PI;
    else if(angle > M_PI)
        angle -= 2.0*M_PI;
    return angle;
}
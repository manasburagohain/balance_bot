/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){
    
    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){
    
    printf("%d\n",rc_pwm_init(1,DEFAULT_PWM_FREQ));
    printf("blah\n");
    printf("%d\n",rc_gpio_init(MOT_BRAKE_EN, GPIOHANDLE_REQUEST_OUTPUT));
    rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    init_flag = 1;
    mb_motor_set_all(0);
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }
    rc_pwm_cleanup(1);
    rc_gpio_cleanup(MDIR1_CHIP, MDIR1_PIN);
    rc_gpio_cleanup(MDIR2_CHIP, MDIR2_PIN);
    init_flag = 0;
    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }

    if(unlikely(rc_gpio_set_value(MOT_BRAKE_EN, brake_en))){
        fprintf(stderr,"ERROR in rc_motor_brake, failed to write to gpio pin %d,%d\n",MDIR1_CHIP, MDIR1_PIN);
        return -1;
    }
   
   return 0;
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
    mb_motor_set_all(0);

    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    int a,b;
    if(motor == 1)
        duty=duty*MOT_1_POL;
    else
        duty=duty*MOT_2_POL;
    
    if(duty>=0.0){  a=1; b=0;}
    else{       a=0; b=1; duty=-duty;}
    
    if(motor == 1)
    {
        rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN, a);
        rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN, b);
        rc_pwm_set_duty(1, 'A', duty);
    }
    else
    {
        rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, a);
        rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, b);
        rc_pwm_set_duty(1, 'B', duty);   
    }
    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    mb_motor_set(1, duty);
    mb_motor_set(2, duty);
    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    // TO DO Read value from ADC
    return 0;

}
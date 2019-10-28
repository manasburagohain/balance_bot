/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "../common/mb_defs.h"


#include "balancebot.h"
static rc_filter_t D1 = RC_FILTER_INITIALIZER;
static rc_filter_t D2 = RC_FILTER_INITIALIZER;
double X_offset;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	if(mb_motor_init()==-1){
		fprintf(stderr,"failed to start initialize Motors\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

	
	FILE *fptr;
	fptr = fopen("../balancebot/params", "r");
	if (fptr == NULL){
		printf("params file not found\n");
		return -1;
	}
	char str[7];
	char name[20]; //trash

	//Read in D1_GAIN
	fgets(str, 7, fptr);
	double D1_GAIN = atof(str);
	fgets(name, 20, fptr);

	//Read in D1_NUM
	double D1_num[3];
	fgets(str, 7, fptr);
	D1_num[0] = atof(str);
	fgets(str, 7, fptr);
	D1_num[1] = atof(str);
	fgets(str, 8, fptr);
	D1_num[2] = atof(str);
	fgets(name, 20, fptr);

	//Read in D1_DEN
	double D1_den[3];
	fgets(str, 6, fptr);
	D1_den[0] = atof(str);
	fgets(str, 8, fptr);
	D1_den[1] = atof(str);
	fgets(str, 7, fptr);
	D1_den[2] = atof(str);
	fgets(name, 20, fptr);



	//Read in D2_GAIN
	fgets(str, 7, fptr);
	double D2_GAIN = atof(str);
	fgets(name, 20, fptr);

	//Read in D2_NUM
	double D2_num[3];
	fgets(str, 7, fptr);
	D2_num[0] = atof(str);
	fgets(str, 9, fptr);
	D2_num[1] = atof(str);
	fgets(str, 8, fptr);
	D2_num[2] = atof(str);
	fgets(name, 20, fptr);

	//Read in D2_DEN
	double D2_den[3];
	fgets(str, 7, fptr);
	D2_den[0] = atof(str);
	fgets(str, 9, fptr);
	D2_den[1] = atof(str);
	fgets(str, 8, fptr);
	D2_den[2] = atof(str);
	fgets(name, 20, fptr);


	//Read in X_offset
	fgets(str, 3, fptr);
	X_offset = atof(str)*3.14/180;
	fclose(fptr);

	printf("D1_GAIN: %f\n",D1_GAIN);
	printf("D1_NUM: %f %f %f\n",D1_num[0],D1_num[1],D1_num[2]);
	printf("D1_DEN: %f %f %f\n",D1_den[0],D1_den[1],D1_den[2]);
	printf("D2_GAIN: %f\n",D2_GAIN);
	printf("D2_NUM: %f %f %f\n",D2_num[0],D2_num[1],D2_num[2]);
	printf("D2_DEN: %f %f %f\n",D2_den[0],D2_den[1],D2_den[2]);
	printf("X_offset: %f\n", X_offset);

	if(rc_filter_alloc_from_arrays(&D1, DT, D1_num, D1_NUM_LEN, D1_den, D1_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
		return -1;
	}
	D1.gain = D1_GAIN;
	rc_filter_enable_saturation(&D1, -1.0, 1.0);
	rc_filter_enable_soft_start(&D1, SOFT_START_SEC);

	// set up D2
	// double D2_num[] = D2_NUM;
	// double D2_den[] = D2_DEN;
	if(rc_filter_alloc_from_arrays(&D2, DT, D2_num, D2_NUM_LEN, D2_den, D2_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D2\n");
		return -1;
	}
	D2.gain = D2_GAIN;
	rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
	rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}


	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	rc_filter_free(&D1);
	rc_filter_free(&D2);
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	static int inner_saturation_counter = 0;
	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
	// Read encoders
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);
    
    double wheelAngleR = (rc_encoder_eqep_read(RIGHT_MOTOR) * 2.0 * M_PI) /(ENC_2_POL * GEAR_RATIO * ENCODER_RES);
    double wheelAngleL = (rc_encoder_eqep_read(LEFT_MOTOR) * 2.0 * M_PI) /(ENC_1_POL * GEAR_RATIO * ENCODER_RES);

    mb_state.phi = (wheelAngleL + wheelAngleR) / 2 + mb_state.theta;


 //    if(0){
	// 	if(fabs(setpoint.phi_dot) > 0.001) setpoint.phi += setpoint.phi_dot*DT;
	// 	cstate.d2_u = rc_filter_march(&D2,setpoint.phi-cstate.phi);
	// 	setpoint.theta = cstate.d2_u;
	// }
	// else setpoint.theta = 0.0;

	/************************************************************
	* OUTER LOOP PHI controller D2
	* Move the position setpoint based on phi_dot.
	* Input to the controller is phi error (setpoint-state).
	*************************************************************/

	
	double d2_u = rc_filter_march(&D2,0-mb_state.phi);
	// setpoint.theta = d2_u;

	/************************************************************
	* INNER LOOP ANGLE Theta controller D1
	* Input to D1 is theta error (setpoint-state). Then scale the
	* output u to compensate for changing battery voltage.
	*************************************************************/
	double d2_u_offset = -d2_u-mb_state.theta;
	//if(d2_u_offset > 0) d2_u_offset += 0.01;
	//else d2_u_offset -= 0.01;
	
	double d1_u = rc_filter_march(&D1,X_offset+d2_u_offset);

	if(fabs(d1_u)>0.95) inner_saturation_counter++;
	else inner_saturation_counter = 0;
	// if saturate for a second, disarm for safety
	if(inner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)){
		printf("inner loop controller saturated\n");
		mb_motor_disable();
		inner_saturation_counter = 0;
		return;
	}

	mb_state.left_cmd =  d1_u ;
	mb_state.right_cmd = d1_u ;
	mb_motor_set(LEFT_MOTOR, -mb_state.left_cmd);
	mb_motor_set(RIGHT_MOTOR, -mb_state.right_cmd);

    // Update odometry 
 	

    // Calculate controller outputs
    
    if(!mb_setpoints.manual_ctl){
    	//send motor commands
   	}

    if(mb_setpoints.manual_ctl){
    	//send motor commands
   	}

	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	
	
   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

	while(1){

		if(rc_dsm_is_new_data()){
				// TODO: Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			//printf("%7.3f  |", -mb_state.left_cmd);
			printf("%7.3f  |", mb_state.phi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 
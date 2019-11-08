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
static rc_filter_t D3 = RC_FILTER_INITIALIZER;
double X_offset;
long int t;
double theta_ref;
double phi_ref;
double gamma_ref;
double setpoint;


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

	// start dsm listener
	if(rc_dsm_init()==-1){
			fprintf(stderr,"failed to start initialize DSM\n");
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

	double goal[3];
	fgets(str, 7, fptr);
	goal[0] = atof(str);
	fgets(str, 9, fptr);
	goal[1] = atof(str);
	fgets(str, 8, fptr);
	goal[2] = atof(str);


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
	printf("goal position: %f %f %f\n",goal[0],goal[1],goal[2]);

	if(rc_filter_alloc_from_arrays(&D1, DT, D1_num, D1_NUM_LEN, D1_den, D1_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
		return -1;
	}
	// if(rc_filter_pid(&D1, D1_KP, D1_KI, D1_KD, D1_WC, DT)){
	// 	fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
	// 	return -1;
	// }
	D1.gain = D1_GAIN;
	rc_filter_enable_saturation(&D1, -1.0, 1.0);
	rc_filter_enable_soft_start(&D1, SOFT_START_SEC);

	// if(rc_filter_pid(&D2, D2_KP, D2_KI, D2_KD, D2_WC, DT)){
	// 	fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
	// 	return -1;
	// }
	// D2.gain = D2_GAIN;
	// rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
	// rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

	// set up D2
	// double D2_num[] = D2_NUM;	

	//double D2_den[] = D2_DEN;
	if(rc_filter_alloc_from_arrays(&D2, DT, D2_num, D2_NUM_LEN, D2_den, D2_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D2\n");
		return -1;
	}
	D2.gain = D2_GAIN;
	rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
	rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

	if(rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
		fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
		return -1;
	}
	rc_filter_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	// printf("starting print thread... \n");
	// pthread_t  printf_thread;
	// rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	//start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);
	
	// printf("starting drive square thread");
	// pthread_t drive_square_control_thread;
	// rc_pthread_create(&drive_square_control_thread, drive_square_control_loop,  (void*) NULL, SCHED_FIFO, 50);

	// printf("starting race thread");
	// pthread_t race_thread;
	// rc_pthread_create(&race_thread, race,  (void*) NULL, SCHED_FIFO, 50);

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

	mb_setpoints.manual_ctl = 1;
	mb_setpoints.phi   = 0.0;
	mb_setpoints.gamma = 0.0;

	// initialize mb_state
	mb_state.left_encoder = 0;
	mb_state.right_encoder = 0;
	mb_state.gamma = mpu_data.dmp_TaitBryan[TB_YAW_Z];
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
	t= 0;
	theta_ref=0.0;
	phi_ref = 0.0;
	gamma_ref=0.0;
	setpoint = 0.0;

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
	rc_filter_free(&D3);
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
	t++;
	// if(t%500==0){
	// 	if(t%1000==0)
	// 		phi_ref = 0.1;
	// 	else
	// 	 	phi_ref = 0;
	// }
	if(t>500){
		gamma_ref = M_PI/2;
	}
	setpoint = 2*phi_ref/WHEEL_DIAMETER;
	
	static int inner_saturation_counter = 0;
	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.dgamma_gyro = mpu_data.dmp_TaitBryan[TB_YAW_Z] - mb_state.gamma_gyro;
	mb_state.gamma_gyro = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	mb_state.d_left = rc_encoder_eqep_read(1)- mb_state.left_encoder;
	mb_state.d_right = rc_encoder_eqep_read(2)- mb_state.right_encoder;

	// Read encoders, changed to encoder values since last readings
	mb_state.left_encoder = rc_encoder_eqep_read(LEFT_MOTOR) ;
	mb_state.right_encoder = rc_encoder_eqep_read(RIGHT_MOTOR);
    
    double wheelAngleR = (rc_encoder_eqep_read(RIGHT_MOTOR) * 2.0 * M_PI) /(ENC_2_POL * GEAR_RATIO * ENCODER_RES);
    double wheelAngleL = (rc_encoder_eqep_read(LEFT_MOTOR) * 2.0 * M_PI) /(ENC_1_POL * GEAR_RATIO * ENCODER_RES);

    mb_state.phi = (wheelAngleL + wheelAngleR) / 2 + mb_state.theta;
	mb_state.gamma = (wheelAngleR-wheelAngleL) * ((WHEEL_DIAMETER/2)/WHEEL_BASE);


	/************************************************************
	* OUTER LOOP PHI controller D2
	* Move the position setpoint based on phi_dot.
	* Input to the controller is phi error (setpoint-state).
	*************************************************************/
	//double d2_u = rc_filter_march(&D2,0-mb_state.phi);
	if(fabs(mb_setpoints.fwd_velocity) > 0.001) mb_setpoints.phi += mb_setpoints.fwd_velocity *DT;
	double d2_u = rc_filter_march(&D2,setpoint-mb_state.phi);

	/************************************************************
	* INNER LOOP ANGLE Theta controller D1
	* Input to D1 is theta error (setpoint-state). Then scale the
	* output u to compensate for changing battery voltage.
	*************************************************************/
	double d2_u_offset = -d2_u-mb_state.theta;
	//if(d2_u_offset > 0) d2_u_offset += 0.01;
	//else d2_u_offset -= 0.01;
	
	double d1_u = rc_filter_march(&D1,X_offset+d2_u_offset);
	//double d1_u = rc_filter_march(&D1, theta_ref-mb_state.theta);
	if(fabs(d1_u)>0.95) inner_saturation_counter++;
	else inner_saturation_counter = 0;
	// if saturate for a second, disarm for safety
	if(inner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)){
		printf("inner loop controller saturated\n");
		mb_motor_disable();
		inner_saturation_counter = 0;
		return;
	}

	/**********************************************************
	* gama (steering) controller D3
	* move the setpoint gamma based on user input like phi
	***********************************************************/
	
	if(fabs(mb_setpoints.turn_velocity)>0.0001) mb_setpoints.gamma += mb_setpoints.turn_velocity * DT;
	//double d3_u = rc_filter_march(&D3,mb_setpoints.gamma - mb_state.gamma);
	double d3_u = rc_filter_march(&D3, gamma_ref - mb_state.gamma);

	/**********************************************************/
    // Update odometry 
 	mb_odometry_update(&mb_odometry, &mb_state);

    // Calculate controller outputs
    
    if(!mb_setpoints.manual_ctl){
    	//send motor commands
		mb_state.left_cmd =  d1_u;
		mb_state.right_cmd = d1_u;
		mb_motor_set(LEFT_MOTOR, -mb_state.left_cmd);
		mb_motor_set(RIGHT_MOTOR, -mb_state.right_cmd);

	}

    if(mb_setpoints.manual_ctl){
    	//send motor commands
		mb_state.left_cmd =  d1_u + d3_u;
		mb_state.right_cmd = d1_u - d3_u;
		mb_motor_set(LEFT_MOTOR, -mb_state.left_cmd);
		mb_motor_set(RIGHT_MOTOR, -mb_state.right_cmd);

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
	//printf("%f, %f\n", phi_ref, mb_state.phi*WHEEL_DIAMETER/2.);
	printf("%f, %f\n", gamma_ref, mb_state.gamma);

   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}

/*******************************************************************************
*  For odometry testing, actually useless
*
*
 void balancebot_controller(){
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.dgamma_gyro = mpu_data.dmp_TaitBryan[TB_YAW_Z] - mb_state.gamma_gyro;
	mb_state.gamma_gyro = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	// Read encoders, changed to encoder values since last readings
	mb_state.d_left = rc_encoder_eqep_read(1)- mb_state.left_encoder;
	mb_state.d_right = rc_encoder_eqep_read(2)- mb_state.right_encoder;
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);
	mb_odometry_update(&mb_odometry, &mb_state);
 }
*******************************************************************************/


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	double drive_stick, turn_stick; // input sticks


	while(1){
		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/SAMPLE_RATE_HZ);

		if(rc_dsm_is_new_data()){
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may should implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.
			// Read normalized (+-1) inputs from RC radio stick and multiply by
			// polarity setting so positive stick means positive setpoint
			turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
			drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;

			// saturate the inputs to avoid possible erratic behavior
			//rc_saturate_double(&drive_stick,-1,1);
			//rc_saturate_double(&turn_stick,-1,1);

			// use a small deadzone to prevent slow drifts in position
			if(fabs(drive_stick)<DSM_DEAD_ZONE) drive_stick = 0.0;
			if(fabs(turn_stick)<DSM_DEAD_ZONE)  turn_stick  = 0.0;
			//printf("Stick: %f %f\n", drive_stick, turn_stick);

			// translate normalized user input to real setpoint values
			
			mb_setpoints.fwd_velocity   = DRIVE_RATE * drive_stick;
			mb_setpoints.turn_velocity =  TURN_RATE * turn_stick;
			//printf("Velocity: %f %f\n", mb_setpoints.fwd_velocity, mb_setpoints.turn_velocity);
		}
		else if(rc_dsm_is_connection_active()==0){
			mb_setpoints.fwd_velocity = 0;
			mb_setpoints.turn_velocity = 0;
			continue;
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}




void* drive_square_control_loop(void* ptr){
	// double Vp = 2.5;
	double V = 2.5;
	double Vt = 1.0;

	while(1){
		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/SAMPLE_RATE_HZ);
		if(fabs(mb_odometry.x)<0.8){
			mb_setpoints.fwd_velocity   = V;
			mb_setpoints.turn_velocity =  0;
		}
		else if(fabs(mb_odometry.gamma + M_PI/2)>0.05){
			//mb_setpoints.fwd_velocity = Vp * (1-mb_odometry.x);
			mb_setpoints.turn_velocity = -Vt;
		}
		else if(fabs(mb_odometry.gamma + M_PI/2)<0.05){
			printf("set point reached %f.\n",(mb_odometry.gamma + M_PI/2));
			mb_setpoints.turn_velocity = 0;
			mb_setpoints.fwd_velocity = 0;
			mb_odometry.x = 0;
			mb_odometry.y = 0;
			mb_odometry.gamma = mb_odometry.gamma + M_PI/2;
		}
		// printf("Turn %f\n", -(mb_odometry.gamma + M_PI/2));
		//printf("Velocity: %f %f\n", mb_setpoints.fwd_velocity, mb_setpoints.turn_velocity);

	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}

void* race(void* ptr)
{
	// double Vp = 11.8;
	// double V = 5;
	// double Vt = 0.5;
	while(1){
		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/SAMPLE_RATE_HZ);
		// if(fabs(mb_odometry.x)<10){
		// 	mb_setpoints.fwd_velocity   = 1.5*mb_odometry.x + 15;
		// 	mb_setpoints.turn_velocity =  0;
		// }
		// else if(fabs(mb_odometry.x)>10 && fabs(mb_odometry.x)<11.5){
		// 	mb_setpoints.fwd_velocity = 20.0 * (11.5-mb_odometry.x);
		// 	mb_setpoints.turn_velocity = 0;
		// }
		// else{
		// 	mb_setpoints.fwd_velocity = 0;
		// 	mb_setpoints.turn_velocity = 0;
		// }
		double xhat = 9;
		double a = 0;
		double b = 0;
		double V0 = 10;
		double L = 11.3;
		double Vmax = 27;
		if(fabs(mb_odometry.x)<xhat){
			a = (Vmax - V0)/sqrt(xhat);
			mb_setpoints.fwd_velocity   = a*sqrt(mb_odometry.x) + V0;
			mb_setpoints.turn_velocity =  0;
		}
		else if(fabs(mb_odometry.x)>xhat && fabs(mb_odometry.x)<L){
			b = Vmax/sqrt(L-xhat);
			mb_setpoints.fwd_velocity = b * sqrt(L-mb_odometry.x);
			mb_setpoints.turn_velocity = 0;
		}
		else{
			mb_setpoints.fwd_velocity = 0;
			mb_setpoints.turn_velocity = 0;
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
			printf("                 SENSORS               |      COMMAND      |          Odometry           |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("  Fwd V  |");
			printf("  Turn V |");
			printf("    x    |");
			printf("    y    |");
			printf("  orient |");
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
			printf("%7.3f  |", mb_setpoints.fwd_velocity);
			printf("%7.3f  |", mb_setpoints.turn_velocity);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_odometry.gamma);
			// printf("\n");
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 
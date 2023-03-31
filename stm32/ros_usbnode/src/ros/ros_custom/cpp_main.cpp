/**
  ******************************************************************************
  * @file    cpp_main.c
  * @author  Georg Swoboda <cn@warp.at>
  * @date    21/09/2022
  * @version 1.0.0
  * @brief   ROS Node main C++ routines
  ******************************************************************************
  * Main ROS routines
  * Publish/Subscribe to Topics
  * Provide Services
  * Odometry (for DR)
  ******************************************************************************
  */


#include "board.h"
#include "main.h"

#include <cpp_main.h>
#include "panel.h"
#include "emergency.h"
#include "drivemotor.h"
#include "blademotor.h"
#include "spiflash.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int16MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

// IMU
#include "imu/imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Temperature.h"
#include <xbot_msgs/WheelTick.h>

// Status message
#include "mower_msgs/Status.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"


#define MAX_MPS	  	0.6		 	// Allow maximum speed of 0.6 m/s 
#define PWM_PER_MPS 300.0		// PWM value of 300 means 1 m/s bot speed

#define TICKS_PER_M 250.0		// Motor Encoder ticks per meter

//#define WHEEL_BASE  0.325		// The distance between the center of the wheels in meters
#define WHEEL_BASE  0.285		// The distance between the center of the wheels in meters
#define WHEEL_DIAMETER 0.198 	// The diameter of the wheels in meters

#define ODOM_NBT_TIME_MS   100 	// 200ms
#define EXT_IMU_NBT_TIME_MS  100
#define ONBOARD_IMU_NBT_TIME_MS  100
#define MOTORS_NBT_TIME_MS 100
#define STATUS_NBT_TIME_MS 250

uint8_t RxBuffer[RxBufferSize];
struct ringbuffer rb;

ros::Time last_cmd_vel(0, 0);
uint32_t last_cmd_vel_age;		// age of last velocity command

// drive motor control
static uint8_t left_speed=0;
static uint8_t right_speed=0;
static uint8_t left_dir=0;
static uint8_t right_dir=0;

#ifdef BLADEMOTOR_USART_ENABLED
	// blade motor control
	static uint8_t blade_on_off=0;
#endif

ros::NodeHandle nh;

float imu_onboard_temperature; // cached temp value, so we dont poll I2C constantly


// IMU
// external IMU (i2c)
sensor_msgs::Imu imu_msg;
// onboard IMU (accelerometer and temp)
sensor_msgs::Imu imu_onboard_msg;
//sensor_msgs::Temperature imu_onboard_temp_msg;

xbot_msgs::WheelTick wheel_ticks_msg;

// om status message
mower_msgs::Status om_mower_status_msg;
/*
 * PUBLISHERS
 */
ros::Publisher pubOMStatus("mower/status", &om_mower_status_msg);
ros::Publisher pubWheelTicks("mower/wheel_ticks", &wheel_ticks_msg);

// IMU onboard
ros::Publisher pubIMUOnboard("imu_onboard/data_raw", &imu_onboard_msg);

// IMU external
ros::Publisher pubIMU("imu/data_raw", &imu_msg);

/*
 * SUBSCRIBERS
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg);
extern "C" void CommandHighLevelStatusMessageCb(const mower_msgs::HighLevelStatus& msg);
ros::Subscriber<geometry_msgs::Twist> subCommandVelocity("cmd_vel", CommandVelocityMessageCb);
ros::Subscriber<mower_msgs::HighLevelStatus> subCommandHighLevelStatus("mower_logic/current_state", CommandHighLevelStatusMessageCb);

// SERVICES
void cbSetEmergency(const mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res);
void cbReboot(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void cbHighLevelControl(const mower_msgs::HighLevelControlSrvRequest &req, mower_msgs::HighLevelControlSrvResponse &res);
#ifdef BLADEMOTOR_USART_ENABLED
	void cbEnableMowerMotor(const mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res);
#endif

ros::ServiceServer<mower_msgs::EmergencyStopSrvRequest, mower_msgs::EmergencyStopSrvResponse> svcSetEmergency("mower_service/emergency", cbSetEmergency);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> svcReboot("mowgli/Reboot", cbReboot);
#ifdef BLADEMOTOR_USART_ENABLED
	ros::ServiceServer<mower_msgs::MowerControlSrvRequest, mower_msgs::MowerControlSrvResponse> svcEnableMowerMotor("mower_service/mow_enabled", cbEnableMowerMotor);
#endif
ros::ServiceClient<mower_msgs::HighLevelControlSrvRequest, mower_msgs::HighLevelControlSrvResponse> svcHighLevelControl("mower_service/high_level_control");
/*
 * NON BLOCKING TIMERS
 */
static nbt_t ros_nbt;
static nbt_t publish_nbt;
static nbt_t motors_nbt;
static nbt_t panel_nbt;
static nbt_t onboard_imu_nbt;
static nbt_t status_nbt;
#ifdef HAS_EXT_IMU
	static nbt_t ext_imu_nbt;
#endif

/*
 * reboot flag, if true we reboot after next publish_nbt
 */
static bool reboot_flag = false;

extern "C" void CommandHighLevelStatusMessageCb(const mower_msgs::HighLevelStatus& msg){
	if (msg.gps_quality_percent < 0.9) {
		PANEL_Set_LED(PANEL_LED_MON, PANEL_LED_OFF);
	} else {
		PANEL_Set_LED(PANEL_LED_MON, PANEL_LED_ON);
	}
}

/*
 * receive and parse cmd_vel messages
 * actual driving (updating drivemotors) is done in the drivemotors_nbt
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg)
{
		last_cmd_vel = nh.now();

		//	debug_printf("x: %f  z: %f\r\n", msg.linear.x, msg.angular.z);

		// calculate twist speeds to add/substract 
		float left_twist_mps = -1.0 * msg.angular.z * WHEEL_BASE * 0.5;
		float right_twist_mps =  msg.angular.z * WHEEL_BASE * 0.5;
    

		// add them to the linear speed 
		float left_mps = msg.linear.x + left_twist_mps;
        float right_mps = msg.linear.x + right_twist_mps;

		// cap left motor speed to MAX_MPS
		if (left_mps > MAX_MPS)
		{
			left_mps = MAX_MPS;
		}
		else if (left_mps < -1.*MAX_MPS)
		{
			left_mps = -1.*MAX_MPS;
		}
		// cap right motor speed to MAX_MPS
		if (right_mps > MAX_MPS)
		{
			right_mps = MAX_MPS;
		}
		else if (right_mps < -1.*MAX_MPS)
		{
			right_mps = -1.*MAX_MPS;
		}

		// set directions		
		left_dir = (left_mps >= 0)?1:0;
		right_dir = (right_mps >= 0)?1:0;

		// set drivemotors PWM values
		left_speed = abs(left_mps * PWM_PER_MPS);
		right_speed = abs(right_mps * PWM_PER_MPS);		

	//	debug_printf("left_mps: %f (%c)  right_mps: %f (%c)\r\n", left_mps, left_dir?'F':'R', right_mps, right_dir?'F':'R');
}

uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len){

	ringbuffer_put(&rb,Buf,len);
	return CDC_RX_DATA_HANDLED;
}


/*
 * Update various chatters topics
 */
extern "C" void chatter_handler()
{
	  if (NBT_handler(&publish_nbt))
	  {
		 
#ifdef IMU_ONBOARD_TEMP
		  imu_onboard_temperature = IMU_TempRaw();
#else
		  imu_onboard_temperature = IMU_Onboard_ReadTemp();
#endif
/*
		  imu_onboard_temp_msg.variance = 0.5;		// 0.5°C resolution
		  imu_onboard_temp_msg.header.frame_id = base_link;
		  pubIMUOnboardTemp.publish(&imu_onboard_temp_msg);
*/
		  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);         // flash LED

		  // reboot if set via cbReboot (mowgli/Reboot)
		  if (reboot_flag)
		  {
			nh.spinOnce();
			NVIC_SystemReset();
			// we never get here ...
		  }
	  }
}

/*
 *  Drive Motors handler
 *  Blade Motor handler
 */
extern "C" void motors_handler()
{
	  if (NBT_handler(&motors_nbt))
	  {
		if (Emergency_State())
		{			
			DRIVEMOTOR_SetSpeed(0,0,0,0);
#ifdef BLADEMOTOR_USART_ENABLED			
			BLADEMOTOR_Set(0);
#endif			
		}
		else {
			// if the last velocity cmd is older than 1sec we stop the drive motors
			last_cmd_vel_age = nh.now().sec - last_cmd_vel.sec;			
			if (last_cmd_vel_age > 1) {
				DRIVEMOTOR_SetSpeed(0, 0, left_dir, right_dir);
			}
			else {
				DRIVEMOTOR_SetSpeed(left_speed, right_speed, left_dir, right_dir);
			}
#ifdef BLADEMOTOR_USART_ENABLED
			// if the last blade cmd is older than 25sec we stop the motor			
			if (last_cmd_vel_age > 25) {
				blade_on_off = 0;				
			}			
			BLADEMOTOR_Set(blade_on_off);			
#endif			
		}
	  }
}

/*
 *  Keyboard/LED Panel handler
 */
extern "C" void panel_handler()
{
	  if (NBT_handler(&panel_nbt))
	  {			  
		PANEL_Tick();
		if (buttonupdated == 1 && buttoncleared == 0)
		{
			debug_printf("ROS: panel_nbt() - buttonstate changed\r\n");
			mower_msgs::HighLevelControlSrvRequest highControlRequest;
			mower_msgs::HighLevelControlSrvResponse highControlResponse;
			if (buttonstate[PANEL_BUTTON_DEF_S1]) {
				highControlRequest.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S1;
			}
			if (buttonstate[PANEL_BUTTON_DEF_S2]) {
				highControlRequest.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S2;
			}
			if (buttonstate[PANEL_BUTTON_DEF_LOCK]) {
				highControlRequest.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY;
			}
			if (buttonstate[PANEL_BUTTON_DEF_SUN]) {
				highControlRequest.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS;
			}
			if (buttonstate[PANEL_BUTTON_DEF_OK]) {
				highControlRequest.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_START;
			}
			svcHighLevelControl.call(highControlRequest, highControlResponse);
			buttonupdated=0;
		}
	  }
}

/* \fn wheelTicks_handler
* \brief Send wheelt tick to openmower by rosserial
* is called when receiving the motors unit answer (every 20ms)
*/
extern "C" void wheelTicks_handler(int8_t p_u8LeftDirection,int8_t p_u8RightDirection, uint32_t p_u16LeftTicks, uint32_t p_u16RightTicks, int16_t p_s16LeftSpeed, int16_t p_s16RightSpeed){

    wheel_ticks_msg.stamp = nh.now();
    wheel_ticks_msg.wheel_tick_factor = TICKS_PER_M;
    wheel_ticks_msg.valid_wheels = 0x0C;
    wheel_ticks_msg.wheel_direction_fl = 0;
    wheel_ticks_msg.wheel_ticks_fl = (int32_t)p_s16LeftSpeed;
    wheel_ticks_msg.wheel_direction_fr = 0;
    wheel_ticks_msg.wheel_ticks_fr = (int32_t)p_s16RightSpeed;
    wheel_ticks_msg.wheel_direction_rl = (p_u8LeftDirection == -1)? 1 : 0;
    wheel_ticks_msg.wheel_ticks_rl = p_u16LeftTicks;
    wheel_ticks_msg.wheel_direction_rr = (p_u8RightDirection == -1)? 1 : 0;;
    wheel_ticks_msg.wheel_ticks_rr = p_u16RightTicks;

    pubWheelTicks.publish(&wheel_ticks_msg);
}

extern "C" void broadcast_handler()
{	

	 
#ifdef HAS_EXT_IMU
	  if (NBT_handler(&ext_imu_nbt))
	  {
		////////////////////////////////////////
		// IMU Messages
		////////////////////////////////////////		
		imu_msg.header.frame_id = "imu";
		
		// No Orientation in IMU message
		imu_msg.orientation.x = 0;
		imu_msg.orientation.y = 0;
		imu_msg.orientation.z = 0;
		imu_msg.orientation.w = 0;
		imu_msg.orientation_covariance[0] = -1;

		/**********************************/
		/* Exernal Accelerometer 		  */
		/**********************************/
#ifdef IMU_ACCELERATION
		// Linear acceleration		
		IMU_ReadAccelerometer(&imu_msg.linear_acceleration.x, &imu_msg.linear_acceleration.y, &imu_msg.linear_acceleration.z);		
		IMU_AccelerometerSetCovariance(imu_msg.linear_acceleration_covariance);	
#else
		imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.z = 0;		
		imu_msg.linear_acceleration_covariance[0] = -1;
#endif

		/**********************************/
		/* Exernal Gyro					  */
		/**********************************/
#ifdef IMU_ANGULAR
		// Angular velocity
		IMU_ReadGyro(&imu_msg.angular_velocity.x, &imu_msg.angular_velocity.y, &imu_msg.angular_velocity.z);
		IMU_GyroSetCovariance(imu_msg.angular_velocity_covariance);	
#else
		imu_msg.angular_velocity.x = imu_msg.angular_velocity.y = imu_msg.angular_velocity.z = 0;		
		imu_msg.angular_velocity_covariance[0] = -1;
#endif		
		imu_msg.header.stamp = nh.now();
		pubIMU.publish(&imu_msg);
	  } // if (NBT_handler(&ext_imu_nbt))
#endif

 	   if (NBT_handler(&onboard_imu_nbt))	  
	  {
		/**********************************/
		/* Onboard (GForce) Accelerometer */
		/**********************************/
#ifdef IMU_ONBOARD_ACCELERATION
		IMU_Onboard_ReadAccelerometer(&imu_onboard_msg.linear_acceleration.x, &imu_onboard_msg.linear_acceleration.y, &imu_onboard_msg.linear_acceleration.z);		
		IMU_Onboard_AccelerometerSetCovariance(imu_onboard_msg.linear_acceleration_covariance);	
#else
		imu_onboard_msg.linear_acceleration.x = imu_onboard_msg.linear_acceleration.y = imu_onboard_msg.linear_acceleration.z = 0;		
#endif
		// no onboard gyro so angular velocities are always zero
		imu_onboard_msg.angular_velocity.x = imu_onboard_msg.angular_velocity.y = imu_onboard_msg.angular_velocity.z = 0;		
		imu_onboard_msg.angular_velocity_covariance[0] = -1;		// indicate *not valid* to EKF
		imu_onboard_msg.header.stamp = nh.now();
		pubIMUOnboard.publish(&imu_onboard_msg);		
	  } // if (NBT_handler(&onboard_imu_nbt))

  	  if (NBT_handler(&status_nbt))
	  {
		////////////////////////////////////////
		// mowgli/status Message
		////////////////////////////////////////		
		/*status_msg.stamp = nh.now();
		status_msg.rain_detected = RAIN_Sense();
        status_msg.emergency_status = Emergency_State();
		status_msg.emergency_tilt_mech_triggered = Emergency_Tilt();
		status_msg.emergency_tilt_accel_triggered = Emergency_LowZAccelerometer();
		status_msg.emergency_left_wheel_lifted = Emergency_WheelLiftBlue();
		status_msg.emergency_right_wheel_lifted = Emergency_WheelLiftRed();
		status_msg.emergency_stopbutton_triggered = Emergency_StopButtonYellow() || Emergency_StopButtonWhite();
		status_msg.left_encoder_ticks = left_encoder_ticks;
		status_msg.right_encoder_ticks = right_encoder_ticks;
		status_msg.v_charge = charge_voltage;
		status_msg.i_charge = charge_current;
		status_msg.v_battery = battery_voltage;
		status_msg.charge_pwm = chargecontrol_pwm_val;
		status_msg.is_charging = chargecontrol_is_charging;
		status_msg.imu_temp = imu_onboard_temperature;
	    status_msg.blade_motor_ctrl_enabled = blade_on_off;
		status_msg.drive_motor_ctrl_enabled = true; // hardcoded for now
		status_msg.blade_motor_enabled = BLADEMOTOR_bActivated;	// set by feedback from blademotor	
		status_msg.left_power = left_power;	
		status_msg.right_power = right_power; 
    	status_msg.blade_power = BLADEMOTOR_u16Power;
	    status_msg.blade_RPM = BLADEMOTOR_u16RPM;
	    status_msg.blade_temperature = blade_temperature;
		status_msg.sw_ver_maj = MOWGLI_SW_VERSION_MAJOR;
		status_msg.sw_ver_bra = MOWGLI_SW_VERSION_BRANCH;
		status_msg.sw_ver_min = MOWGLI_SW_VERSION_MINOR;*/
		//pubStatus.publish(&status_msg);		

		om_mower_status_msg.rain_detected = RAIN_Sense();
		om_mower_status_msg.emergency = Emergency_State();
		/* not used anymore*/
		if (chargecontrol_is_charging) {
			om_mower_status_msg.v_charge = 32.0;
		} else {
			om_mower_status_msg.v_charge = 0.0;
		}
		om_mower_status_msg.charge_current = charge_current;
		om_mower_status_msg.v_battery = battery_voltage;
		om_mower_status_msg.left_esc_status.current = left_power;
		om_mower_status_msg.right_esc_status.current = right_power;
		om_mower_status_msg.mow_esc_status.temperature_motor = blade_temperature;
		om_mower_status_msg.mow_esc_status.tacho = BLADEMOTOR_u16RPM;
		om_mower_status_msg.mow_esc_status.current = BLADEMOTOR_u16Power;
		om_mower_status_msg.mow_esc_status.status = BLADEMOTOR_bActivated;
		om_mower_status_msg.left_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
		om_mower_status_msg.right_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;

		pubOMStatus.publish(&om_mower_status_msg);
	  } // if (NBT_handler(&status_nbt))
}

/*
 *  callback for mowgli/EnableMowerMotor Service
 */
#ifdef BLADEMOTOR_USART_ENABLED
	void cbEnableMowerMotor(const mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res)
	{	
		debug_printf("ROS: cbEnableMowerMotor(from %d to %d)", blade_on_off, req.mow_enabled);	
		blade_on_off = req.mow_enabled;	
		debug_printf("[DONE]\r\n");
	}
#endif

/*
 * callback for reset emergency state by remote
 */
void cbSetEmergency(const mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res)
{
	Emergency_SetState(req.emergency);
}

/*
 *  callback for mowgli/Reboot Service
 */
void cbReboot(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	//debug_printf("cbReboot:\r\n");
	reboot_flag = true;	
}

/*
 * ROS housekeeping
 */
extern "C" void spinOnce()
{
	  if (NBT_handler(&ros_nbt))
	  {
			nh.spinOnce();
	  }
}

/* 
 *  Initialize rosserial
 */
extern "C" void init_ROS()
{
	ringbuffer_init(&rb, RxBuffer, RxBufferSize);

	// Initialize ROS
	nh.initNode();

	// Initialize Pubs
#ifdef HAS_EXT_IMU	
	nh.advertise(pubIMU);
#endif
    nh.advertise(pubWheelTicks);
	nh.advertise(pubIMUOnboard);
	nh.advertise(pubOMStatus);
	
	// Initialize Subscribers
	nh.subscribe(subCommandVelocity);
	nh.subscribe(subCommandHighLevelStatus);

	// Initialize Services	
#ifdef BLADEMOTOR_USART_ENABLED	
    nh.advertiseService(svcEnableMowerMotor);
#endif
    nh.advertiseService(svcSetEmergency);
	nh.advertiseService(svcReboot);
	nh.serviceClient(svcHighLevelControl);
	
	// Initialize Timers
	NBT_init(&publish_nbt, 1000);
	NBT_init(&panel_nbt, 100);	
	NBT_init(&status_nbt, STATUS_NBT_TIME_MS);
#ifdef HAS_EXT_IMU	
	NBT_init(&ext_imu_nbt, EXT_IMU_NBT_TIME_MS);
#endif	
	NBT_init(&onboard_imu_nbt, ONBOARD_IMU_NBT_TIME_MS);

	NBT_init(&motors_nbt, MOTORS_NBT_TIME_MS);
	NBT_init(&ros_nbt, 10);	
}
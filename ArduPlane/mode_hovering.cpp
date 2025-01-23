#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_WindVane/AP_WindVane.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_InertialNav/AP_InertialNav.h>

#include "mode.h"
#include "Plane.h"
#include "zef_control.h"

using namespace AP;

// Serial manager is needed for UART communications
static AP_SerialManager serial_manager;

Location desired_position;
bool must_reset_loc;
double ret_errors[3];
NavEKF3 ekf3;

AP_InertialNav zef_inertial_nav = AP_InertialNav(AP::ahrs());

bool ModeHovering::_enter()
{


    // Initialize the UART for GPS system
    //serial_manager.init();
    ret_errors[0] = 0.0;
    ret_errors[1] = 0.0;
    ret_errors[2] = 0.0;
	int16_t min_motor = 1000;
    int16_t max_motor = 2000; 
	int16_t angle_min = 500;
    int16_t angle_max = 2500;
    //gps().init(serial_manager);
	
    gps().update();
    must_reset_loc = true; //semaforo para resetar posição

    zefiroControl.set_manual(false);

    return true;
}

void ModeHovering::update()
{
    static uint32_t last_msg_ms;
    
    gps().update(); // Update GPS state based on possible bytes received from the module.
    AP::ahrs().update();
    
	is_armed();
	Location loc;
	update_error_position();
	wind_vane_available();
	get_wind_speed();
	get_3d_position();
  // If new GPS data is received, output its contents to the console
    // Here we rely on the time of the message in GPS class and the time of last message
    // saved in static variable last_msg_ms. When new message is received, the time
    // in GPS class will be updated.
    

    
    Vector3f inertial_vector((float) ret_errors[0], (float) ret_errors[1], (float) ret_errors[2]);
    Vector3f body_vector = zefiroControl.rotate_inertial_to_body(roll, pitch, yaw, inertial_vector);
    
    zefiroControl.set_RHO(plane.barometer.get_pressure()*0.00001);

    /*if (barometer.healthy()) {
        //output barometer readings to console
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, " Pressure: %.2f Pa\n"
                            " Temperature: %.2f degC\n"
                            " Relative Altitude: %.2f m\n"
                            " climb=%.2f m/s\n"
                            "\n",
                            (double)barometer.get_pressure(),
                            (double)barometer.get_temperature(),
                            (double)barometer.get_altitude(),
                            (double)barometer.get_climb_rate());
    }*/

    //plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pitch: %f --- yaw: %f --- roll: %f", pitch, yaw, roll);

    // Read angular rates for roll, pitch, and yaw

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang_pitch: %f --- ang_yaw: %f --- ang_roll: %f", angular_pitch, angular_yaw, angular_roll);

    //zefiroControl.update(double U_longit_speed, double V_lateral_speed, double W_vertical_speed,
    //    double P_v_roll, double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw);
    //double wind_direction = wrap_PI(plane.g2.windvane.get_apparent_wind_speeddirection_rad()); // radians(wrap_360(degrees(plane.g2.windvane.get_apparent_wind_direction_rad())));

    zefiroControl.update(U_longit_speed, V_lateral_speed, W_vertical_speed,
        P_v_roll, Q_v_pitch, R_v_yaw,
        roll, pitch, yaw,
        body_vector.x, body_vector.y, body_vector.z); //loc.lat, loc.lng, loc.alt);

    //double d1_angel_motor, d2_angel_motor, d3_angel_motor, d4_angel_motor;
    //double dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo, dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo;
    int16_t output_d1_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d1_angel_motor, angle_min, angle_max);
    int16_t output_d2_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d2_angel_motor, angle_min, angle_max);
    int16_t output_d3_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d3_angel_motor, angle_min, angle_max);
    int16_t output_d4_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d4_angel_motor, angle_min, angle_max);

    SRV_Channels::move_servo(SRV_Channel::k_motor_tilt, output_d1_angel, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearRight, output_d2_angel, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRear, output_d3_angel, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearLeft, output_d4_angel, angle_min, angle_max);

    //zefiroControl.operateInflators(&plane.barometer, plane.g2.min_balloon_pressure_diff, plane.g2.max_balloon_pressure_diff);
}	
	
void ModeHovering::_exit(){
    /*for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->disable_ch(i);
    }*/
    //zefiroControl.stopInflators();
}

bool is_armed(){
    return AP::arming().is_armed();
}

		/*int f1 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.f1_forca_motor, min_motor, max_motor);
		int f2 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.f2_forca_motor, min_motor, max_motor);
		int f3 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.f3_forca_motor, min_motor, max_motor);
		int f4 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.f4_forca_motor, min_motor, max_motor);*/

		int f1 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.comando_m1, min_motor, max_motor);
		int f2 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.comando_m2, min_motor, max_motor);
		int f3 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.comando_m3, min_motor, max_motor);
		int f4 = zefirocontrol.get_value_to_pwm_motor(zefirocontrol.comando_m4, min_motor, max_motor);

		srv_channels::move_servo(srv_channel::k_motor1, f1, min_motor, max_motor);
		srv_channels::move_servo(srv_channel::k_motor2, f2, min_motor, max_motor);
		srv_channels::move_servo(srv_channel::k_motor3, f3, min_motor, max_motor);
		srv_channels::move_servo(srv_channel::k_motor4, f4, min_motor, max_motor);
	} else {
		srv_channels::move_servo(srv_channel::k_motor1, 0, min_motor, max_motor);
		srv_channels::move_servo(srv_channel::k_motor2, 0, min_motor, max_motor);
		srv_channels::move_servo(srv_channel::k_motor3, 0, min_motor, max_motor);
		srv_channels::move_servo(srv_channel::k_motor4, 0, min_motor, max_motor);
	}
}
    
void update_error_position(){
    static uint32_t last_msg_ms = 0;
 if (last_msg_ms != gps().last_message_time_ms()) {
		// Reset the time of message
		last_msg_ms = gps().last_message_time_ms();
		if( must_reset_loc ) {
			must_reset_loc = false;
			
			AP::ahrs().EKF3.getLLH(desired_position); //desired_position = gps().location();
		}

		// Acquire location
		//const Location &loc = gps().location();
		AP::ahrs().EKF3.getLLH(loc);
		//float ground_speed = gps().ground_speed(); // Get ground speed in meters per second*/

		// Print the contents of message
		/*GCS_SEND_TEXT(MAV_SEVERITY_INFO, " Lat: %.2fm \n"
										" Lon: %.2fm \n"
										" Alt: %.2fm \n"
										" Ground speed: %.2fm \n",
										(double)loc.lat,
										(double)loc.lng,
										(double)(loc.alt * 0.01f),
										(double)ground_speed);*/

		zefiroControl.getPositionError((double) desired_position.lat, (double) desired_position.lng, (double)(desired_position.alt), (double) loc.lat, (double) loc.lng, (double)(loc.alt), (double) yaw, ret_errors);
	}

}	
	
void wind_vane_available() {
	if (AP::windvane() == nullptr) {
		GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Windvane module not available");
	}
}

void get_3d_position(){
	double U_longit_speed = vx;
	double V_lateral_speed = vy;
	double W_vertical_speed = plane.barometer.get_climb_rate();
	double P_v_roll = angular_roll;
	double Q_v_pitch = angular_pitch;
	double R_v_yaw = angular_yaw;
}

void get_update_location(){
	float pitch = AP::ahrs().get_pitch(); // Get pitch angle in degrees
    float yaw = AP::ahrs().get_yaw(); // Get yaw angle in degrees
    float roll = AP::ahrs().get_roll(); // Get roll angle in degrees
	
    Vector3f angular_gyro = AP::ahrs().get_gyro();
    float angular_pitch = angular_gyro.y; // Get angular rate for pitch in degrees per second
    float angular_roll = angular_gyro.x; // Get angular rate for roll in degrees per second
    float angular_yaw = angular_gyro.z; // Get angular rate for yaw in degrees per second 
}

void get_wind_speed(){
	double wind_direction = wrap_PI(AP::windvane()->get_apparent_wind_direction_rad());
    double wind_speed = AP::windvane()->get_apparent_wind_speed();
    double vx = wind_speed * cosf(wind_direction);
    double vy = wind_speed * sinf(wind_direction);
}

	/* Quaternion test code
	    GCS_SEND_TEXT(MAV_SEVERITY_INFO,    " x: %.4fm \n" #teste
                                        " y: %.4fm \n"
                                        " z: %.4fm \n",
                                        (double)body_vector.x,
                                        (double)body_vector.y,
                                        (double)body_vector.z);

    //plane.barometer.accumulate();
    //plane.barometer.update();
	float listaTeste[10][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
    {1, 1, 0},
    {1, 0, 1},
    {0, 1, 1},
    {1, 1, 1},
    {2, 0, 0},
    {0, 2, 0},
    {0, 0, 2}
    };


   	for (int i=0; i<10;++i){
		float t_roll = listaTeste[i][0];
		float t_pitch=listaTeste[i][1];
		float t_yaw=listaTeste[i][2];
 
	enum Rotation rotation = ROTATION_NONE;

	const float q_accuracy = 1.0e-3f;
	Quaternion q, qe;
	q.from_rotation(rotation);
	qe.from_euler(radians(t_roll), radians(t_pitch), radians(t_yaw));

	float q_roll, q_pitch, q_yaw, qe_roll, qe_pitch, qe_yaw;
	q.to_euler(q_roll, q_pitch, q_yaw);
	qe.to_euler(qe_roll, qe_pitch, qe_yaw);

	float roll_diff = fabsf(wrap_PI(q_roll - qe_roll));
	float pitch_diff = fabsf(wrap_PI(q_pitch - qe_pitch));
	float yaw_diff = fabsf(wrap_PI(q_yaw - qe_yaw));

	if ((roll_diff > q_accuracy) || (pitch_diff > q_accuracy) || (yaw_diff > q_accuracy)) {
		hal.console->printf("quaternion test %u failed : yaw:%f/%f roll:%f/%f pitch:%f/%f\n",
		(unsigned)rotation,
		(double)q_yaw, (double)qe_yaw,
		(double)q_roll, (double)qe_roll,
		(double)q_pitch, (double)qe_pitch);

		GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Yaw: %.4f\n"
										   "Roll: %.4f\n"
										   "Pitch: %.4f\n",
										   (double)qe_yaw,
										   (double)qe_roll,
										   (double)qe_pitch);
	}
    }*/
  //controles da empenagem
    /*int16_t output_vert_stabilizer = zefiroControl.get_value_to_pwm_servo(zefiroControl.dvu_ang_estab_vert_cima, angle_min, angle_max);
    int16_t output_hor_stabilizer = zefiroControl.get_value_to_pwm_servo(zefiroControl.dhr_ang_estab_horiz_direito, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_rudder, output_vert_stabilizer, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_elevator, output_hor_stabilizer, angle_min, angle_max);*/
	
	
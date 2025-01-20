#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#define HAL_USE_I2C     TRUE
#define HAL_USE_I2S     FALSE
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/Display.h>
#include "Plane.h"
#include "zef_control.h"

//Zef_control.cpp alterado

using namespace AP;

void ZefControl::mult_mat(const float matrix[12][12], const double vector[12], double (&results)[12]) {
    int rows = 12;
    int cols = 12;
    for (int i = 0; i < rows; i++) {
        results[i] = 0.0;
        for (int j = 0; j < cols; j++) {
            results[i] += ((double)matrix[i][j]) * vector[j];          
}
 
void ZefControl::find_matrix(const double longit_speed) {
    int max_options = sizeof(speed_refs);
    ref_index = max_options - 1;
    for (int i = max_options-1; i >= 0; i--) 
        if (longit_speed < speed_refs[i]) 
            ref_index = i;
 } 
    int len = sizeof(k_matrix_all) / sizeof(k_matrix_all[0]);
    if(ref_index > len-1) ref_index = len-1; 
}

void ZefControl::add_traction(double (&U_array)[12], double longit_speed) {
    double Cd = 0.03;
    double Volume_2_3 = 9.0;
    for (int i = 0; i < 8; i++) {
        if(i % 2 == 0) {
          
  double f_static = 0.5*RHO_air_density*Cd*Volume_2_3*(powf(longit_speed,2)/4);
    U_array[i] += f_static;      
 }
    
    U_array[0] += antagonist_force;
    U_array[2] += antagonist_force*-1;
    U_array[4] += antagonist_force;
    U_array[6] += antagonist_force*-1;
}

void ZefControl::adjust_for_manual(double (&U_array)[12]) {

    double ajuste_cal_roll_mot = 0.3;
    double ajuste_cal_pitch_mot = 0.6;
    double ajuste_cal_yaw_mot = 0.6;
    double ajuste_cal_cima_baixo = 0.6;
    double ajuste_cal_roll_estab = 0.3;
    double ajuste_cal_dir_esq = 0.3;
    double center_value = 0.0;

	U_array[0] = J3_cmd_throttle + ajuste_cal_pitch_mot*J2_cmd_pitch;
	U_array[1] = -ajuste_cal_dir_esq*RS_cmd_up_down;

	U_array[2] = J3_cmd_throttle - ajuste_cal_yaw_mot*J4_cmd_yaw - ajuste_cal_pitch_mot*J2_cmd_pitch/2;
	U_array[3] = ajuste_cal_cima_baixo*LS_cmd_letf_right + ajuste_cal_roll_mot*J1_cmd_roll;

	U_array[4] = J3_cmd_throttle - ajuste_cal_pitch_mot*J2_cmd_pitch;
	U_array[5] = ajuste_cal_dir_esq*RS_cmd_up_down + ajuste_cal_roll_mot*J1_cmd_roll;

	U_array[6] = J3_cmd_throttle + ajuste_cal_yaw_mot*J4_cmd_yaw - ajuste_cal_pitch_mot*J2_cmd_pitch/2;
	U_array[7] = -ajuste_cal_cima_baixo*LS_cmd_letf_right + ajuste_cal_roll_mot*J1_cmd_roll;

	U_array[8] = center_value + J4_cmd_yaw + (ajuste_cal_roll_estab*J1_cmd_roll); 
	U_array[9] = center_value + J2_cmd_pitch + (ajuste_cal_roll_estab*J1_cmd_roll); 

	U_array[10] = center_value + (-J4_cmd_yaw) + (ajuste_cal_roll_estab*J1_cmd_roll); 
	U_array[11] = center_value + (-J2_cmd_pitch) + (ajuste_cal_roll_estab*J1_cmd_roll); 

    for(int i = 0; i < 8; i++) {
        U_array[i] *= F_max_mot;
    }

void ZefControl::manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch) {
    double multiplicador_comandos = 1.0; //4500;
    double multiplicador_motor = 1.0; //100;
    J1_cmd_roll = aileron/multiplicador_comandos;
    J2_cmd_pitch = elevator/multiplicador_comandos;
    J3_cmd_throttle = throttle/multiplicador_motor;
    J4_cmd_yaw = rudder/multiplicador_comandos;
    RS_cmd_up_down = right_switch;
    LS_cmd_letf_right = left_switch;

}

double ZefControl::set_angle_range(double last_angle, double v1, double v0) {
    double pi = 3.14159265358979311600;
    double Ang_max = 210/180*pi;
    double d_angel_motor_ant = last_angle;
    double d_angel_motor = atan2f(v1, v0);
    if (d_angel_motor>(2*pi-Ang_max) && d_angel_motor_ant < -(2*pi-Ang_max) ) {
        d_angel_motor =  -(2*pi-d_angel_motor);
    }
    if (d_angel_motor<-(2*pi-Ang_max) && d_angel_motor_ant > (2*pi-Ang_max) ) {
        d_angel_motor =  (2*pi-d_angel_motor);
    }
    return d_angel_motor;
}

double ZefControl::get_engine_command(double motor_force) {
    double command = 0.0;
    if( motor_force >= F_min_mot ) {
        if (motor_force > F_max_mot) {
            command = comando_maximo;
        } else {
            command =  (coeficiente_quadratico_forca * powf(motor_force, 2)) + (coeficiente_linear_forca * motor_force) + coeficiente_nulo_forca;
        }
    }

    return command;
}

void ZefControl::set_power_and_angles(double (&U)[12]) {
	double ang_anterior = 0.0;
    double coef_antec_mov = 0.5;
    double f_min_servo = F_min_mot * coef_antec_mov;

    F1_forca_motor = sqrtf(powf(U[0], 2) + powf(U[1], 2));
    if( F1_forca_motor >= f_min_servo ) {
        ang_anterior = d1_angel_motor;
        d1_angel_motor = set_angle_range(ang_anterior, U[1], U[0]);
    }

    F2_forca_motor = sqrtf(powf(U[2], 2) + powf(U[3], 2));
    if( F2_forca_motor >= f_min_servo ) {
        ang_anterior = d2_angel_motor;
        d2_angel_motor = set_angle_range(ang_anterior, U[3], U[2]);
    }

    F3_forca_motor = sqrtf(powf(U[4], 2) + powf(U[5], 2));
    if( F3_forca_motor >= f_min_servo ) {
        ang_anterior = d3_angel_motor;
        d3_angel_motor = set_angle_range(ang_anterior, U[5], U[4]);
    }

    F4_forca_motor = sqrtf(powf(U[6], 2) + powf(U[7], 2));
    if( F4_forca_motor >= f_min_servo ) {
        ang_anterior = d4_angel_motor;
        d4_angel_motor = set_angle_range(ang_anterior, U[7], U[6]);
    }

    dvu_ang_estab_vert_cima = U[8];
    dvd_ang_estab_vert_baixo = U[9];

    dhr_ang_estab_horiz_direito = U[10];
    dhl_ang_estab_horiz_esquerdo = U[11];


void ZefControl::put_forces_in_range() {
 
    comando_M1 = get_engine_command(F1_forca_motor);
    comando_M2 = get_engine_command(F2_forca_motor);
    comando_M3 = get_engine_command(F3_forca_motor);
    comando_M4 = get_engine_command(F4_forca_motor);
}

void ZefControl::print_output_data() {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F1: %f -- ang: %f", F1_forca_motor, d1_angel_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F2: %f -- ang: %f", F2_forca_motor, d2_angel_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F3: %f -- ang: %f", F3_forca_motor, d3_angel_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F4: %f -- ang: %f", F4_forca_motor, d4_angel_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang v cima: %f -- ang v baixo: %f", dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang h dir: %f -- ang h esq: %f", dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo);
}

double sign(double x);
double min(double x, double y);

double sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

double min(double x, double y) {
    if(x < y) return x;
    if(y < x) return y;
    return x;
}

void ZefControl::update(
        double U_longit_speed, double V_lateral_speed, double W_vertical_speed,
        double P_v_roll, double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw,
        double x_position_n_s, double y_position_e_w, double z_position_height) {
    
    if(is_manual_mode) {
        double U[12];
        adjust_for_manual(U);
        set_power_and_angles(U); 
        put_forces_in_range();
       
    } else {
        double X[12] = {
            U_longit_speed,
            V_lateral_speed,
            W_vertical_speed,
            P_v_roll,
            Q_v_pitch,
            R_v_yaw,
            Roll,
            Pitch,
            Yaw,
            x_position_n_s,
            y_position_e_w,
            z_position_height,
        };

        double w_speed = 0.0;
        double pi = 3.141592653589793;
        if (sign(U_longit_speed)*powf(U_longit_speed,2)/(powf(U_longit_speed,2)+powf(V_lateral_speed,2)+powf(W_vertical_speed,2))>cosf(30/180*pi)) {
            w_speed = U_longit_speed;
        }
        if(plane.g.matrix_index == -1) {
            find_matrix(w_speed);
        } else {
            int max_option = sizeof(speed_refs) - 1;
            int curr_index = plane.g.matrix_index;
            if(curr_index > max_option) curr_index = max_option;
            find_matrix(curr_index);
        }

        double U[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double vetor_erro[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double X_ref[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double alpha = 0.2;
        for( int i = 0; i < 12; i++) {
            X[i] = (last_X[i] * (1-alpha)) + (X[i] * alpha);
        }
        for( int i = 0; i < 9; i++) {
            vetor_erro[i] = X_ref[i] - X[i];
        }
		
        for ( int i = 9; i < 12; i++) {
            vetor_erro[i] = X_ref[i] - X[i];
        }


        mult_mat(k_matrix_all[ref_index], vetor_erro, U);
        memcpy(last_X, X, sizeof(last_X));

        }*/
        add_traction(U, U_longit_speed);
        set_power_and_angles(U);
        put_forces_in_range(); 
    }
}

void ZefControl::operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit) {
double pressure_diff = (barometer->get_pressure(1) - barometer->get_pressure(0)) * 0.01;

    if ( pressure_diff < min_p && inflator_state == 0 ) { 
        inflator_state = 1;
        hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(gpio_pin, 1);
        hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
        hal.gpio->write(gpio_pin+1, 1);
    } else {
        if ( inflator_state == 1 && pressure_diff > max_p ) {
            inflator_state = 0;
        }
        if ( inflator_state == 1 ) {
            hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin, 1);
            hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin+1, 1);
        } else {
            hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin, 0);
            hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
            hal.gpio->write(gpio_pin+1, 0);
        }
    }
   
}
void ZefControl::stopInflators(){
    inflator_state = 0;
    hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(gpio_pin, 0);
    hal.gpio->pinMode(gpio_pin+1, HAL_GPIO_OUTPUT);
    hal.gpio->write(gpio_pin+1, 0);
}
Vector3f ZefControl::rotate_inertial_to_body(float roll, float pitch, float yaw, const Vector3f &inertial_vector) {
    Quaternion q;
    q.from_euler(roll, pitch, yaw);
    Vector3f body_vector = inertial_vector;
    q.earth_to_body(body_vector);

    return body_vector;
}
void ZefControl::getPositionError(double desired_position_lat, double desired_position_long, double desired_position_alt, double current_position_lat, double current_position_long, double current_position_alt, double azimute, double (&ret_errors)[3]) {
    double earth_radius = 6.371e6;
    double convert_degrees_to_radians = 0.01745329252;
    double adjust_gps_reading = 1e-7;
    double error_x_earth_ref = convert_degrees_to_radians * ( desired_position_lat - current_position_lat ) * adjust_gps_reading * earth_radius;
    double error_y_earth_ref =  convert_degrees_to_radians * ( desired_position_long - current_position_long ) * adjust_gps_reading * earth_radius * cosf ( convert_degrees_to_radians * current_position_lat );
    double error_z_earth_ref = - ( desired_position_alt - current_position_alt )*0.01;
    double error_x_airship_ref = sinf( azimute ) * error_x_earth_ref + cosf( azimute ) *  error_y_earth_ref;
    double error_y_airship_ref = cosf( azimute ) * error_x_earth_ref - sinf( azimute ) *  error_y_earth_ref;
    double error_z_airship_ref = error_z_earth_ref;
    ret_errors[0] = error_x_airship_ref;
    ret_errors[1] = error_y_airship_ref;
    ret_errors[2] = error_z_airship_ref;
}
}
ZefControl zefiroControl;

// Declarações no zef_control.h

void ZefControl::TCA9548A(uint16_t bus) {
    if (tca_initialized == 0) {
        tca_initialized = 1;
        // Lógica para a função TCA9548A
    }
}

void ZefControl::set_manual(bool state) {
    is_manual_mode = state;
}

void ZefControl::set_RHO(double rho) {
    if (rho > 0.0) {
        RHO_air_density = rho;
    } else {
        RHO_air_density = 1.2;
    }
}

float ZefControl::dead_zone(double input_value, double dead_zone_limit) {
    double retVal = input_value;
    if ((input_value < dead_zone_limit) && (input_value > -dead_zone_limit))
        retVal = 0.0;
    return retVal;
}

int ZefControl::get_value_to_pwm_servo(double in_value, int min_pwm, int max_pwm) {
    int range = (max_pwm - min_pwm) / 2;
    int center_value = (min_pwm + max_pwm) / 2;
    int retVal = ((double)(in_value / 3.14159265) * range) + center_value;
    if (retVal > max_pwm) retVal = max_pwm;
    if (retVal < min_pwm) retVal = min_pwm;
    return retVal;
}

int ZefControl::get_value_to_pwm_motor(double in_value, int min_pwm, int max_pwm) {
    int range = (max_pwm - min_pwm);
    int retVal = (in_value * range) + min_pwm;
    if (retVal > max_pwm) retVal = max_pwm;
    if (retVal < min_pwm) retVal = min_pwm;
    return retVal;
}

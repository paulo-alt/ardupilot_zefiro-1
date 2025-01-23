#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_IO_MCU || CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>

#include "mode.h"
#include "Plane.h"
#include "zef_control.h"

using namespace AP;

static AP_SerialManager serial_manager;

int16_t angle_min = 500;
int16_t angle_max = 2500;
int16_t min_motor = 1000;
int16_t max_motor = 2000;

bool ModeManualK::_enter() {
    initialize_rc_channels();
    zefiroControl.set_manual(true);
    initialize_motors();
    return true;
}

void ModeManualK::_exit() {
    disable_motors();
}

void ModeManualK::update() {
    gps().update();
    AP::ahrs().update();

    get_update_location();
    get_3d_position();
    handle_barometer();
    handle_wind();
    handle_rc_inputs();
    handle_zefiro_control();
    handle_servos();
    handle_arming_state();
}

// ===========================
// Funções Modulares Separadas
// ===========================

void initialize_rc_channels() {
    rc().channel(CH_5)->set_range(1900);
    rc().channel(CH_6)->set_range(1900);
}

void initialize_motors() {
    for (int i = 0; i < 4; i++) {
        SRV_Channels::set_output_scaled((SRV_Channel::k_motor1 + i), 1500);
    }
}

void disable_motors() {
    for (int i = 0; i < 4; i++) {
        SRV_Channels::move_servo((SRV_Channel::k_motor1 + i), 0, min_motor, max_motor);
    }
}

void get_update_location() {
    float local_pitch = AP::ahrs().get_pitch();
    float local_yaw = AP::ahrs().get_yaw();
    float local_roll = AP::ahrs().get_roll();

    Vector3f angular_gyro = AP::ahrs().get_gyro();
    float angular_pitch = angular_gyro.y;
    float angular_roll = angular_gyro.x;
    float angular_yaw = angular_gyro.z;
}

void get_3d_position() {
    double U_longit_speed = AP::ahrs().get_velocity().x;
    double V_lateral_speed = AP::ahrs().get_velocity().y;
    double W_vertical_speed = plane.barometer.get_climb_rate();
    double P_v_roll = AP::ahrs().get_gyro().x;
    double Q_v_pitch = AP::ahrs().get_gyro().y;
    double R_v_yaw = AP::ahrs().get_gyro().z;
}

void handle_barometer() {
    plane.barometer.update();
    zefiroControl.set_RHO(plane.barometer.get_pressure());
}

void handle_wind() {
    if (AP::windvane() != nullptr) {
        Vector3f wind = AP::windvane()->get_wind();
        double vx = -wind.x;
        double vy = -wind.y;
    }
}

void handle_rc_inputs() {
    double control_aileron = rc().channel(CH_1)->get_control_in();
    double control_pitch = rc().channel(CH_2)->get_control_in();
    double control_yaw = rc().channel(CH_4)->get_control_in();
    double control_throttle = rc().channel(CH_3)->get_control_in();
    double control_right_switch = rc().channel(CH_5)->get_control_in();
    double control_left_switch = rc().channel(CH_6)->get_control_in();

    int normalizador_inputs_apy = 4500;
    int normalizador_throttle = 50;
    int zero_throttle = 50;
    int normalizador_aux = 1900 / 2;
    int zero_aux = 1900 / 2;

    control_aileron /= normalizador_inputs_apy;
    control_pitch /= normalizador_inputs_apy;
    control_yaw /= normalizador_inputs_apy;
    control_throttle = (control_throttle - zero_throttle) / normalizador_throttle;
    control_left_switch = (control_left_switch - zero_aux) / normalizador_aux;
    control_right_switch = (control_right_switch - zero_aux) / normalizador_aux;

    double dead_zone = 0.05;
    control_aileron = zefiroControl.dead_zone(control_aileron, dead_zone);
    control_pitch = zefiroControl.dead_zone(control_pitch, dead_zone);
    control_yaw = zefiroControl.dead_zone(control_yaw, dead_zone);
    control_throttle = zefiroControl.dead_zone(control_throttle, dead_zone);
    control_left_switch = zefiroControl.dead_zone(control_left_switch, dead_zone);
    control_right_switch = zefiroControl.dead_zone(control_right_switch, dead_zone);

    zefiroControl.manual_inputs_update(control_aileron, control_pitch, control_yaw, control_throttle, control_right_switch, control_left_switch);
}

void handle_zefiro_control() {
    zefiroControl.update(
        AP::ahrs().get_velocity().x,
        AP::ahrs().get_velocity().y,
        plane.barometer.get_climb_rate(),
        AP::ahrs().get_gyro().x,
        AP::ahrs().get_gyro().y,
        AP::ahrs().get_gyro().z,
        AP::ahrs().get_roll(),
        AP::ahrs().get_pitch(),
        AP::ahrs().get_yaw(),
        0.0, 0.0, 0.0
    );
}

void handle_servos() {
    int16_t output_d1_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d1_angel_motor, angle_min, angle_max);
    int16_t output_d2_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d2_angel_motor, angle_min, angle_max);
    int16_t output_d3_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d3_angel_motor, angle_min, angle_max);
    int16_t output_d4_angel = zefiroControl.get_value_to_pwm_servo(zefiroControl.d4_angel_motor, angle_min, angle_max);

    SRV_Channels::move_servo(SRV_Channel::k_motor_tilt, output_d1_angel, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearRight, output_d2_angel, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRear, output_d3_angel, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearLeft, output_d4_angel, angle_min, angle_max);
}

void handle_arming_state() {
    if (AP::arming().is_armed()) {
        for (int i = 0; i < 4; i++) {
            int F = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M1 + i, min_motor, max_motor);
            SRV_Channels::move_servo((SRV_Channel::k_motor1 + i), F, min_motor, max_motor);
        }
    } else {
        disable_motors();
    }
}
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

using namespace AP;

// ==============================
// Correção: Fechamento da função mult_mat
// ==============================
void ZefControl::mult_mat(const float matrix[12][12], const double vector[12], double (&results)[12]) {
    int rows = 12;
    int cols = 12;
    for (int i = 0; i < rows; i++) {
        results[i] = 0.0;
        for (int j = 0; j < cols; j++) {
            results[i] += ((double)matrix[i][j]) * vector[j];
        }  // <-- Correção: Adicionado fechamento do loop interno
    }  // <-- Correção: Adicionado fechamento do loop externo
}

// ==============================
// Correção: `sizeof(speed_refs)` retorna bytes, precisa dividir pelo tamanho do elemento
// ==============================
void ZefControl::find_matrix(const double longit_speed) {
    int max_options = sizeof(speed_refs) / sizeof(speed_refs[0]);  // Corrigido para obter o número real de elementos
    ref_index = max_options - 1;
    
    for (int i = max_options - 1; i >= 0; i--) {
        if (longit_speed < speed_refs[i]) {
            ref_index = i;
        }
    }

    int len = sizeof(k_matrix_all) / sizeof(k_matrix_all[0]);
    if (ref_index > len - 1) {
        ref_index = len - 1;
    }
}

// ==============================
// Correção: Fechamento de chave na função `adjust_for_manual`
// ==============================
void ZefControl::adjust_for_manual(double (&U_array)[12]) {
    double ajuste_cal_roll_mot = 0.3;
    double ajuste_cal_pitch_mot = 0.6;
    double ajuste_cal_yaw_mot = 0.6;
    double ajuste_cal_cima_baixo = 0.6;
    double ajuste_cal_roll_estab = 0.3;
    double ajuste_cal_dir_esq = 0.3;
    double center_value = 0.0;

    U_array[0] = J3_cmd_throttle + ajuste_cal_pitch_mot * J2_cmd_pitch;
    U_array[1] = -ajuste_cal_dir_esq * RS_cmd_up_down;

    U_array[2] = J3_cmd_throttle - ajuste_cal_yaw_mot * J4_cmd_yaw - ajuste_cal_pitch_mot * J2_cmd_pitch / 2;
    U_array[3] = ajuste_cal_cima_baixo * LS_cmd_letf_right + ajuste_cal_roll_mot * J1_cmd_roll;

    U_array[4] = J3_cmd_throttle - ajuste_cal_pitch_mot * J2_cmd_pitch;
    U_array[5] = ajuste_cal_dir_esq * RS_cmd_up_down + ajuste_cal_roll_mot * J1_cmd_roll;

    U_array[6] = J3_cmd_throttle + ajuste_cal_yaw_mot * J4_cmd_yaw - ajuste_cal_pitch_mot * J2_cmd_pitch / 2;
    U_array[7] = -ajuste_cal_cima_baixo * LS_cmd_letf_right + ajuste_cal_roll_mot * J1_cmd_roll;

    U_array[8] = center_value + J4_cmd_yaw + (ajuste_cal_roll_estab * J1_cmd_roll);
    U_array[9] = center_value + J2_cmd_pitch + (ajuste_cal_roll_estab * J1_cmd_roll);

    U_array[10] = center_value + (-J4_cmd_yaw) + (ajuste_cal_roll_estab * J1_cmd_roll);
    U_array[11] = center_value + (-J2_cmd_pitch) + (ajuste_cal_roll_estab * J1_cmd_roll);

    for (int i = 0; i < 8; i++) {
        U_array[i] *= F_max_mot;
    }
} // <-- Correção: Fechamento da função

// ==============================
// Correção: Fechamento de chave na função `set_power_and_angles`
// ==============================
void ZefControl::set_power_and_angles(double (&U)[12]) {
    double ang_anterior = 0.0;
    double coef_antec_mov = 0.5;
    double f_min_servo = F_min_mot * coef_antec_mov;

    F1_forca_motor = sqrtf(powf(U[0], 2) + powf(U[1], 2));
    if (F1_forca_motor >= f_min_servo) {
        ang_anterior = d1_angel_motor;
        d1_angel_motor = set_angle_range(ang_anterior, U[1], U[0]);
    }

    F2_forca_motor = sqrtf(powf(U[2], 2) + powf(U[3], 2));
    if (F2_forca_motor >= f_min_servo) {
        ang_anterior = d2_angel_motor;
        d2_angel_motor = set_angle_range(ang_anterior, U[3], U[2]);
    }

    F3_forca_motor = sqrtf(powf(U[4], 2) + powf(U[5], 2));
    if (F3_forca_motor >= f_min_servo) {
        ang_anterior = d3_angel_motor;
        d3_angel_motor = set_angle_range(ang_anterior, U[5], U[4]);
    }

    F4_forca_motor = sqrtf(powf(U[6], 2) + powf(U[7], 2));
    if (F4_forca_motor >= f_min_servo) {
        ang_anterior = d4_angel_motor;
        d4_angel_motor = set_angle_range(ang_anterior, U[7], U[6]);
    }

    dvu_ang_estab_vert_cima = U[8];
    dvd_ang_estab_vert_baixo = U[9];
    dhr_ang_estab_horiz_direito = U[10];
    dhl_ang_estab_horiz_esquerdo = U[11];
} // <-- Correção: Fechamento da função

// ==============================
// Correção: Adicionada verificação `nullptr` antes de acessar o barômetro
// ==============================
void ZefControl::operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit) {
    if (barometer == nullptr) return;  // <-- Adicionada verificação de ponteiro nulo

    double pressure_diff = (barometer->get_pressure(1) - barometer->get_pressure(0)) * 0.01;

    if (pressure_diff < min_p && inflator_state == 0) { 
        inflator_state = 1;
        hal.gpio->pinMode(gpio_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(gpio_pin, 1);
        hal.gpio->pinMode(gpio_pin + 1, HAL_GPIO_OUTPUT);
        hal.gpio->write(gpio_pin + 1, 1);
    } else if (inflator_state == 1 && pressure_diff > max_p) {
        inflator_state = 0;
    }

    hal.gpio->write(gpio_pin, inflator_state);
    hal.gpio->write(gpio_pin + 1, inflator_state);
}
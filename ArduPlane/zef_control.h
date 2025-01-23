#include <math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include "Plane.h" //#include "quadplane.h"
#include "zef_k_matrix.h"

// zef_control.h alterado

class ZefControl {
private:
    // Parâmetros internos do sistema
    double antagonist_force = 3.0E+00;
    double comando_minimo = 2.0E-01; 
    double comando_maximo = 1.0E+00;
    double F_max_mot = 2.1E+01; 
    double F_min_mot = 3.0E-01;
    double coeficiente_quadratico_forca = -1.68E-04;
    double coeficiente_linear_forca = 4.19E-02; 
    double coeficiente_nulo_forca = 1.87E-01;  

    int ref_index = 0; // Índice de referência da matriz K
    double speed_refs[40] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0,31.0,32.0,33.0,34.0,35.0,36.0,37.0,38.0,39.0}; // Speeds references to change the active K matrix
    double RHO_air_density = 1.2; // Densidade do ar para cálculos aerodinâmicos

    // Estado dos infladores
    int gpio_pin = 50;
    int inflator_state = 0;
    int inflator1_state = 0; 
    int inflator2_state = 0;

    float k_matrix_all[40][12][12] = {}; // Matriz de controle K para diferentes velocidades

    // Métodos internos para controle da matriz K
    void find_matrix(const double longit_speed);
    void set_K_matrix(double matrix[12][9]);
    void add_traction(double (&U_array)[12], double longit_speed);

    bool is_manual_mode = false; // Estado do modo manual

    // Métodos auxiliares privados
    void adjust_for_manual(double (&U_array)[12]);
    double set_angle_range(double last_angle, double v1, double v0);
    double get_engine_command(double power);
    void set_power_and_angles(double (&U)[12]);
    void put_forces_in_range();
    void print_output_data();

    // Última posição armazenada
    double last_X[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,};

    int semaforo = 0;

    // Motores e ângulos agora são privados
    double F1_forca_motor, F2_forca_motor, F3_forca_motor, F4_forca_motor;
    double comando_M1, comando_M2, comando_M3, comando_M4;
    double d1_angulo_motor, d2_angulo_motor, d3_angulo_motor, d4_angulo_motor;
    double dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo, dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo;

public:
    // Permissão para outras classes acessarem partes internas, se necessário
    friend class Parameters;
    friend class ParametersG2;
    friend class Plane;

    // Comandos do controle remoto
    double J1_cmd_roll, J2_cmd_pitch, J3_cmd_throttle, J4_cmd_yaw, RS_cmd_up_down, LS_cmd_letf_right = 0.0;

    // Multiplicação de matriz (pública para cálculos externos)
    void mult_mat(const float matrix[8][12], const double vector[12], double (&results)[12]);

    // Controle do modo manual
    void set_manual(bool state); // alterado
    void set_RHO(double rho); // alterado

    // Atualização dos inputs manuais
    void manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch);

    // Atualização geral do sistema
    void update( double U_longit_speed,
        double V_lateral_speed, double W_vertical_speed, double P_v_roll, 
        double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw,
        double x_position_n_s, double y_position_e_w, double z_position_height);
    
    // Conversão de valores para PWM (mantidos públicos para controle externo)
    int get_value_to_pwm_servo(double in_value, int min_pwm, int max_pwm); // alterado
    int get_value_to_pwm_motor(double in_value, int min_pwm, int max_pwm); // alterado

    // Aplicação de zona morta nos controles
    float dead_zone(double input_value, double dead_zone_limit); // alterado

    // Cálculo de erro de posição baseado em coordenadas GPS
    void getPositionError(double desired_position_lat, double desired_position_long, double desired_position_alt, double current_position_lat, double current_position_long, double current_position_alt, double azimute, double (&ret_errors)[3]);

    // Rotação de um vetor do referencial inercial para o referencial do corpo
    Vector3f rotate_inertial_to_body(float roll, float pitch, float yaw, const Vector3f &inertial_vector);

    // Estado do barramento TCA9548A
    int tca_initialized = 0;
    void TCA9548A(uint16_t bus);

    // Construtor - inicializa a matriz K com valores pré-definidos
    ZefControl() {
        memcpy(k_matrix_all, zef_matrix, sizeof(zef_matrix)); // Não sei se é necessário alterar, pois a matriz é carregada de um arquivo externo
    }
};

// Definição de variável global do controlador
extern ZefControl zefiroControl;
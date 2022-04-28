#ifndef PINS_HPP
#define PINS_HPP

#define EMITTER_ON_TIME 2

#define EMIT_L_PIN 21 //D1
#define EMIT_R_PIN 18 //D3
#define EMIT_FL_PIN 22 //D0
#define EMIT_FR_PIN 15 //D2

#define RECIVER_L_PIN 20 //Q1
#define RECIVER_R_PIN 19 //Q3
#define RECIVER_FL_PIN 23 //Q0
#define RECIVER_FR_PIN 14 //Q2

#define M1_BACK_PIN 8
#define M1_FWD_PIN 11
#define M1_SPD_PIN 12
#define M1_ENC_A_PIN 17
#define M1_ENC_B_PIN 16

#define M2_BACK_PIN 9
#define M2_FWD_PIN 7
#define M2_SPD_PIN 6
#define M2_ENC_A_PIN 1
#define M2_ENC_B_PIN 0

unsigned int enc_a_l_count = 0;
unsigned int enc_b_l_count = 0;
unsigned int enc_a_r_count = 0;
unsigned int enc_b_r_count = 0;

#define BUZZ_PIN 2
#define SW_1_PIN 13
#define LED1_PIN 5
#define LED2_PIN 3
#define LED3_PIN 4

#define UPPER_MOTOR_LIMIT 140
#define LOWER_MOTOR_LIMIT 30
#define PID_POLLING_DELAY 15
#define MOTOR_SWITCH_DIR_DELAY 0

#endif
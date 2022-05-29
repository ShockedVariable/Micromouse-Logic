#include <Arduino.h>
#include <PID_v1.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"

#define DEBUG 1

// By-product of using static member variables. Could this be implemented better? Check note in header file.
// For now, it works fine as long as we do not instanitate another MicroMouse object.
volatile unsigned int MicroMouse::enc_backwards_l_count = 0;
volatile unsigned int MicroMouse::enc_forwards_l_count = 0;
volatile unsigned int MicroMouse::enc_backwards_r_count = 0;
volatile unsigned int MicroMouse::enc_forwards_r_count = 0;
int MicroMouse::center = 0;

const unsigned int ticks_to_move = 169;
// 180 appears to be 85 ticks?
const unsigned int turn_ticks = 42;

const float k_p_enc = 0.5f;
// const float k_i_enc = 0.0f;
const float k_d_enc = 0.01f;

const float k_p_ir = 0.08f;
// const float k_i_ir = 0.0f;
const float k_d_ir = 0.005f;

const int l_spd_motor = 125;
const int r_spd_motor = 125;

MicroMouse::MicroMouse(const short& x, const short& y, const Direction& dir)
    : x_pos{x}, y_pos{y}, dir{dir}
{ 
}

void MicroMouse::initConnections()
{
    // Setting baud rate.
    Serial.begin(9600);

    // Emitter pins are set to output.
    pinMode(EMIT_L_PIN, OUTPUT);
    pinMode(EMIT_R_PIN, OUTPUT);
    pinMode(EMIT_FL_PIN, OUTPUT);
    pinMode(EMIT_FR_PIN, OUTPUT);

    // Documentations goes here.
    pinMode(RECEIVER_L_PIN, INPUT);
    pinMode(RECEIVER_R_PIN, INPUT);
    pinMode(RECEIVER_FL_PIN, INPUT);
    pinMode(RECEIVER_FR_PIN, INPUT);

    // Motor 1 pins are set to output
    pinMode(M1_BACK_PIN, OUTPUT);
    pinMode(M1_FWD_PIN, OUTPUT);
    pinMode(M1_SPD_PIN, OUTPUT);
    pinMode(M1_ENC_A_PIN, INPUT); //check if input pullup
    pinMode(M1_ENC_B_PIN, INPUT);

    // Motor 2 pins are set to output
    pinMode(M2_BACK_PIN, OUTPUT);
    pinMode(M2_FWD_PIN, OUTPUT);
    pinMode(M2_SPD_PIN, OUTPUT);
    pinMode(M2_ENC_A_PIN, INPUT); //check if input pullup
    pinMode(M2_ENC_B_PIN, INPUT);

    // Buzzer pin is set to output
    pinMode(BUZZ_PIN, OUTPUT);
    
    pinMode(SW_1_PIN, INPUT);
}

void MicroMouse::attachInterrupts()
{
    // All interrupts caused by the encoder will call the respective function
    // to increment the respective counter.
    attachInterrupt(M2_ENC_A_PIN, enc_backwards_l_intr_handler, FALLING); // check if rising or falling
    attachInterrupt(M2_ENC_B_PIN, enc_forwards_l_intr_handler, FALLING);
    attachInterrupt(M1_ENC_A_PIN, enc_backwards_r_intr_handler, FALLING);
    attachInterrupt(M1_ENC_B_PIN, enc_forwards_r_intr_handler, FALLING);
}

void MicroMouse::findCenter()
{
    const Dists d = getDistRL();
    center = d.r - d.l;
}

int MicroMouse::PID_IR(int& historal_err, int& prev_error)
{
    Dists d = getDistRL();
    // Serial7.printf("Right: %d\r\n", d.r);
    // Serial7.printf("Left: %d\r\n", d.l);
    int sensor_delta = d.r - d.l;
    Serial7.printf("Center: %d\r\n", center);
    Serial7.printf("Delta: %d\r\n", sensor_delta);

    // The error needed for PID calculation. Based on only left and right sensors.
    int error_ir = center - sensor_delta;

    // The integration required for I in PID. This approxmiates integrals by doing Riemann Sums.
    // historal_err += error_ir * elapsed_time;

    // The derivative required for D in PID. This approxmiates the derivative by doing differences/deltas.
    int error_ir_delta = error_ir - prev_error;

    // The proportional part required for P in PID which is the actual value.
    int p_error_ir = k_p_ir * error_ir;

    // The actual I in PID.
    // int i_error_ir = k_i_ir * historal_err;

    // The actual D in PID.
    int d_error_ir = k_d_ir * error_ir_delta;

    int correction = p_error_ir + d_error_ir;

    prev_error = error_ir;

    return correction;
}

int MicroMouse::PID_enc(int& historal_err, int& prev_error)
{
    // The error needed for PID calculation. Based on only left and right encoders.
    int error_enc = static_cast<int>(enc_forwards_r_count) - static_cast<int>(enc_forwards_l_count);

    // The integration required for I in PID. This approxmiates integrals by doing Riemann Sums.
    // historal_err += error_enc * elapsed_time;

    // The derivative required for D in PID. This approxmiates the derivative by doing differences/deltas.
    int error_enc_delta = error_enc - prev_error;

    // The proportional part required for P in PID which is the actual value.
    int p_error_enc = k_p_enc * error_enc;
    
    // The actual I in PID.
    // int i_error_enc = k_i_enc * historal_err;

    // The actual D in PID.
    int d_error_enc = k_d_enc * error_enc_delta;

    int correction = p_error_enc + d_error_enc;

    prev_error = error_enc;

    return correction;
}

void MicroMouse::enc_backwards_l_intr_handler()
{
    ++enc_backwards_l_count;
}

void MicroMouse::enc_forwards_l_intr_handler()
{
    ++enc_forwards_l_count;
    // Serial.printf("Left: %d\r\n", enc_forwards_l_count);
}

void MicroMouse::enc_backwards_r_intr_handler()
{
    ++enc_backwards_r_count;
}

void MicroMouse::enc_forwards_r_intr_handler()
{
    ++enc_forwards_r_count;
    Serial.printf("Right %d\r\n", enc_forwards_r_count);
}

int MicroMouse::getDistL()
{
    int dist;

    // Turns on the emitter, wait for some time, read the numbers reported and store.
    // Turns off the emitter after storing.
    digitalWrite(EMIT_L_PIN, HIGH);
    delay(EMITTER_ON_TIME);
    dist = analogRead(RECEIVER_L_PIN);
    digitalWrite(EMIT_L_PIN, LOW);


    return dist;
}

int MicroMouse::getDistR()
{
    int dist;

    // Turns on the emitter, wait for some time, read the numbers reported and store.
    // Turns off the emitter after storing.
    digitalWrite(EMIT_R_PIN, HIGH);
    delay(EMITTER_ON_TIME);
    dist = analogRead(RECEIVER_R_PIN);
    digitalWrite(EMIT_R_PIN, LOW);

    return dist;
}

Dists MicroMouse::getDistRL()
{
    digitalWrite(EMIT_L_PIN, HIGH);
    digitalWrite(EMIT_R_PIN, HIGH);
    delay(EMITTER_ON_TIME);

    _d.r = analogRead(RECEIVER_R_PIN);
    _d.l = analogRead(RECEIVER_L_PIN);

    return _d;
}

int MicroMouse::getDistFR()
{
    int dist;

    // Turns on the emitter, wait for some time, read the numbers reported and store.
    // Turns off the emitter after storing.
    digitalWrite(EMIT_FR_PIN, HIGH);
    delay(EMITTER_ON_TIME);
    dist = analogRead(RECEIVER_FR_PIN);
    digitalWrite(EMIT_FR_PIN, LOW);

    return dist;
}

int MicroMouse::getDistFL()
{
    int dist;

    // Turns on the emitter, wait for some time, read the numbers reported and store.
    // Turns off the emitter after storing.
    digitalWrite(EMIT_FL_PIN, HIGH);
    delay(EMITTER_ON_TIME);
    dist = analogRead(RECEIVER_FL_PIN);
    digitalWrite(EMIT_FL_PIN, LOW);

    return dist;
}

void MicroMouse::turnAllEmittersOn()
{
    digitalWrite(EMIT_FL_PIN, HIGH);
    digitalWrite(EMIT_FR_PIN, HIGH);
    digitalWrite(EMIT_L_PIN, HIGH);
    digitalWrite(EMIT_R_PIN, HIGH);
}

void MicroMouse::turnAllEmittersOff()
{
    digitalWrite(EMIT_FL_PIN, LOW);
    digitalWrite(EMIT_FR_PIN, LOW);
    digitalWrite(EMIT_L_PIN, LOW);
    digitalWrite(EMIT_R_PIN, LOW);
}

void MicroMouse::turnAllLedOn()
{
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
}

void MicroMouse::turnAllLedOff()
{
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
}

void MicroMouse::setMotorL(const Direction& dir, const int& mspeed)
{    
    switch(dir)
	{
		case Direction::FORWARDS:
			digitalWriteFast(M2_FWD_PIN, LOW);
			digitalWriteFast(M2_BACK_PIN, HIGH);
			analogWrite(M2_SPD_PIN, mspeed);
			break;
		case Direction::BACKWARDS:
			digitalWriteFast(M2_FWD_PIN, HIGH);
			digitalWriteFast(M2_BACK_PIN, LOW);
			analogWrite(M2_SPD_PIN, mspeed);
			break;
		case Direction::STOP:
			digitalWriteFast(M2_FWD_PIN, LOW);
			digitalWriteFast(M2_BACK_PIN, LOW);
			analogWrite(M2_SPD_PIN, 0);
			break;
	}
}

void MicroMouse::setMotorR(const Direction& dir, const int& mspeed)
{
    switch(dir)
	{
		case Direction::FORWARDS:
			digitalWriteFast(M1_BACK_PIN, LOW);
			digitalWriteFast(M1_FWD_PIN, HIGH);
			analogWrite(M1_SPD_PIN, mspeed);
			break;
		case Direction::BACKWARDS:
			digitalWriteFast(M1_FWD_PIN, LOW);
			digitalWriteFast(M1_BACK_PIN, HIGH);
			analogWrite(M1_SPD_PIN, mspeed);
			break;
		case Direction::STOP:
			digitalWriteFast(M1_FWD_PIN, LOW);
			digitalWriteFast(M1_BACK_PIN, LOW);
            analogWrite(M1_SPD_PIN, 0);
			break;
	}
}
// TODO: figure out what this function does 
void MicroMouse::setMotorLPulseDir(const Direction& dir, const int& mspeed)
{
    switch(dir)
    {
        case Direction::FORWARDS:
            digitalWrite(M2_FWD_PIN, LOW);
            digitalWrite(M2_SPD_PIN, HIGH);
            // Why are we pwming the back pin.
            analogWrite(M2_BACK_PIN, mspeed);
            break;
        case Direction::BACKWARDS:
            digitalWrite(M2_BACK_PIN, LOW);
            digitalWrite(M2_SPD_PIN, HIGH);
            analogWrite(M2_FWD_PIN, mspeed);
            break;
        case Direction::STOP:
            digitalWrite(M2_FWD_PIN, LOW);
            digitalWrite(M2_BACK_PIN, LOW);
            analogWrite(M2_SPD_PIN, 0);
            break;
    }
}

void MicroMouse::setMotorRPulseDir(const Direction& dir, const int& mspeed)
{
    switch(dir)
    {
        case Direction::FORWARDS:
            digitalWrite(M1_BACK_PIN, LOW);
            digitalWrite(M1_SPD_PIN, HIGH);
            analogWrite(M1_FWD_PIN, mspeed);
            break;
        case Direction::BACKWARDS:
            digitalWrite(M1_FWD_PIN, LOW);
            digitalWrite(M1_SPD_PIN, HIGH);
            analogWrite(M1_BACK_PIN, mspeed);
            break;

        case Direction::STOP:
            analogWrite(M1_SPD_PIN, 0);
            digitalWrite(M1_FWD_PIN, LOW);
            digitalWrite(M1_BACK_PIN, LOW);
            break;
    }
}

unsigned int MicroMouse::enc_backwards_l_val()
{
    return enc_backwards_l_count;
}

unsigned int MicroMouse::enc_forwards_l_val()
{
    return enc_forwards_l_count;
}

unsigned int MicroMouse::enc_backwards_r_val()
{
    return enc_backwards_r_count;
}

unsigned int MicroMouse::enc_forwards_r_val()
{
    return enc_forwards_r_count;
}

void MicroMouse::rst_enc_backwards_l_counter()
{
    enc_backwards_l_count = 0;
}

void MicroMouse::rst_enc_forwards_l_counter()
{
    enc_forwards_l_count = 0;
}

void MicroMouse::rst_enc_backwards_r_counter()
{
    enc_backwards_r_count = 0;
}

void MicroMouse::rst_enc_forwards_r_counter()
{
    enc_forwards_r_count = 0;
}

void MicroMouse::rstAllEncCounters()
{
    rst_enc_backwards_l_counter();
    rst_enc_backwards_r_counter();
    rst_enc_forwards_l_counter();
    rst_enc_forwards_r_counter();
}

void MicroMouse::goForward(const int& blocks)
{
    setMotorL(FORWARDS, l_spd_motor);
    setMotorR(FORWARDS, 0);
    setMotorR(FORWARDS, r_spd_motor);

    const unsigned int to_move = ticks_to_move * blocks;

    int old_error_ir = 0;
    int old_error_enc = 0;
    int hist_error_ir = 0;
    int hist_error_enc = 0;

    while (enc_forwards_l_count <= to_move && enc_forwards_r_count <= to_move)
    {
        // Also known as the steer values.
        int pid_ir_result = PID_IR(hist_error_ir, old_error_ir);
        int pid_enc_result = PID_enc(hist_error_enc, old_error_enc);

        int proposed_l_spd = l_spd_motor + pid_enc_result;
        int proposed_r_spd = r_spd_motor - pid_enc_result;

        // proposed_l_spd += pid_ir_result;
        // proposed_r_spd -= pid_ir_result;

        if (proposed_l_spd > UPPER_MOTOR_LIMIT)
        {
            proposed_l_spd = UPPER_MOTOR_LIMIT;
        }
        else if (proposed_l_spd < LOWER_MOTOR_LIMIT)
        {
            proposed_l_spd = LOWER_MOTOR_LIMIT;
            digitalWriteFast(BUZZ_PIN, HIGH);
        }
        else
        {
            digitalWriteFast(BUZZ_PIN, LOW);
        }

        if (proposed_r_spd > UPPER_MOTOR_LIMIT)
        {
            proposed_r_spd = UPPER_MOTOR_LIMIT;
        }
        else if (proposed_r_spd < LOWER_MOTOR_LIMIT)
        {
            proposed_r_spd = LOWER_MOTOR_LIMIT;
            digitalWriteFast(BUZZ_PIN, HIGH);
        }
        else
        {
            digitalWriteFast(BUZZ_PIN, LOW);
        }

        setMotorL(FORWARDS, proposed_l_spd);
        setMotorR(FORWARDS, proposed_r_spd);

    }

    setMotorL(STOP, 0);
    setMotorR(STOP, 0);

    rstAllEncCounters();
}

void MicroMouse::turnRight(const int& blocks)
{
    setMotorL(FORWARDS, l_spd_motor);
    setMotorR(BACKWARDS, 0);
    setMotorR(BACKWARDS, r_spd_motor);

    const unsigned int to_move = turn_ticks * blocks;

    while (enc_forwards_l_count <= to_move || enc_backwards_r_count <= to_move);
    {
        // if (enc_forwards_l_count <= to_move && enc_backwards_r_count <= to_move)
        // {
        //     setMotorL(FORWARDS, l_spd_motor);
        //     setMotorR(BACKWARDS, r_spd_motor);
        // }
        if (enc_forwards_l_count <= to_move && enc_backwards_r_count > to_move)
        {
            setMotorL(FORWARDS, l_spd_motor);
            setMotorR(STOP, 0);
        }
        else if (enc_backwards_r_count <= to_move && enc_forwards_l_count > to_move)
        {
            setMotorL(STOP, 0);
            setMotorR(BACKWARDS, r_spd_motor);
        }
        Serial7.printf("Left: %d\r\n", enc_backwards_l_count);
        Serial7.printf("Right: %d\r\n", enc_forwards_r_count);
    };

    setMotorL(STOP, 0);
    setMotorR(STOP, 0);

    rstAllEncCounters();
}

void MicroMouse::turnLeft(const int& blocks)
{
    setMotorL(BACKWARDS, l_spd_motor);
    setMotorR(FORWARDS, 0);
    setMotorR(FORWARDS, r_spd_motor);

    const unsigned int to_move = turn_ticks * blocks;

    while (enc_backwards_l_count <= to_move || enc_forwards_r_count <= to_move)
    {
        // if (enc_backwards_l_count <= to_move && enc_forwards_r_count <= to_move)
        // {

        // }
        if (enc_backwards_l_count <= to_move && enc_forwards_r_count > to_move)
        {
            setMotorL(BACKWARDS, l_spd_motor);
            setMotorR(FORWARDS, 0);
        }
        else if (enc_backwards_l_count > to_move && enc_forwards_r_count <= to_move)
        {
            setMotorL(STOP, 0);
            setMotorR(FORWARDS, r_spd_motor);
        }
    }   

    setMotorL(STOP, 0);
    setMotorR(STOP, 0);

    rstAllEncCounters();
}
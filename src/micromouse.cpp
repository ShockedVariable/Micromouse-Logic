#include <Arduino.h>
#include <PID_v1.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"

#define DEBUG 1

// By-product of using static member variables. Could this be implemented better? Check note in header file.
// For now, it works fine as long as we do not instanitate another MicroMouse object.
unsigned int MicroMouse::enc_a_l_count = 0;
unsigned int MicroMouse::enc_b_l_count = 0;
unsigned int MicroMouse::enc_a_r_count = 0;
unsigned int MicroMouse::enc_b_r_count = 0;
int MicroMouse::center = 0;

const unsigned int ticks_to_move = 169;
const unsigned int turn_ticks = 85;

// const float k_p = 0.0125;
const float k_p = 0.015;
// const float k_p = 0.05;
// const float k_p = 0.2;
// const float k_p = 0.9;
const float k_i = 0;
const float k_d = 0.03;

const int l_spd_motor = 105;
const int r_spd_motor = 105;

// This is corralated to how often we increment our timer back in main.cpp.
// Currently sampling every 1 ms.
const unsigned short SAMPLE_RATE = 1000;

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
    attachInterrupt(M2_ENC_A_PIN, enc_a_l_intr_handler, FALLING); // check if rising or falling
    attachInterrupt(M2_ENC_B_PIN, enc_b_l_intr_handler, FALLING);
    attachInterrupt(M1_ENC_A_PIN, enc_a_r_intr_handler, FALLING);
    attachInterrupt(M1_ENC_B_PIN, enc_b_r_intr_handler, FALLING);
}

void MicroMouse::findCenter()
{
    center = getDistR() - getDistL();
}

int MicroMouse::getCenter()
{
    return center;
}

int MicroMouse::PID(const int& sensor_data, int& historal_err, const unsigned int& elapsed_time, int& prev_error)
{   
    // The error needed for PID calculation. Based on only left and right sensors.
    int error = getCenter() - sensor_data;

    // The integration required for I in PID. This approxmiates integrals by doing Riemann Sums.
    historal_err += error * elapsed_time;

    // The derivative required for D in PID. This approxmiates the derivative by doing differences/deltas.
    int error_delta = error - prev_error;

    Serial7.printf("Error: %d\r\n", error);
    Serial7.printf("Error Delta: %d\r\n", error_delta);

    // The proportional part required for P in PID which is the actual value.
    int p_error = k_p * error;

    // The actual I in PID.
    int i_error = k_i * historal_err;

    // The actual D in PID.
    int d_error = k_d * error_delta;

    int correction = p_error + i_error + d_error;

    prev_error = error;

    Serial7.printf("Correcton: %d\r\n", correction);

    return correction;
}

void MicroMouse::enc_a_l_intr_handler()
{
    ++enc_a_l_count;
}

void MicroMouse::enc_b_l_intr_handler()
{
    ++enc_b_l_count;
}

void MicroMouse::enc_a_r_intr_handler()
{
    ++enc_a_r_count;
}

void MicroMouse::enc_b_r_intr_handler()
{
    ++enc_b_r_count;
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

unsigned int MicroMouse::enc_a_l_val()
{
    return enc_a_l_count;
}

unsigned int MicroMouse::enc_b_l_val()
{
    return enc_b_l_count;
}

unsigned int MicroMouse::enc_a_r_val()
{
    return enc_a_r_count;
}

unsigned int MicroMouse::enc_b_r_val()
{
    return enc_b_r_count;
}

void MicroMouse::rst_enc_a_l_counter()
{
    enc_a_l_count = 0;
}

void MicroMouse::rst_enc_b_l_counter()
{
    enc_b_l_count = 0;
}

void MicroMouse::rst_enc_a_r_counter()
{
    enc_a_r_count = 0;
}

void MicroMouse::rst_enc_b_r_counter()
{
    enc_b_r_count = 0;
}

void MicroMouse::rstAllEncCounters()
{
    rst_enc_a_l_counter();
    rst_enc_a_r_counter();
    rst_enc_b_l_counter();
    rst_enc_b_r_counter();
}

void MicroMouse::goForward(unsigned int& curr_time, const int& blocks)
{
    setMotorL(FORWARDS, l_spd_motor);
    setMotorR(FORWARDS, 0);
    setMotorR(FORWARDS, r_spd_motor);

    unsigned int time = curr_time;

    int hist_error = 0;

    const unsigned int to_move = ticks_to_move * blocks;

    unsigned int b_l_val = enc_b_l_val();
    unsigned int b_r_val = enc_b_r_val();

    int old_error = 0;
    // Serial7.printf("%d\r\n", b_l_val);

    while (b_l_val <= to_move || b_r_val <= to_move)
    {
        Serial7.printf("%d\r\n", b_l_val);

        if (curr_time - time >= SAMPLE_RATE)
        {
            const unsigned int elapsed_time = curr_time - time;
            time = curr_time;

            int r = getDistR();
            int l = getDistL();
            int curr_delta_dist = r - l;

            Serial7.printf("L Sensor Reading: %d\r\n", l);
            Serial7.printf("R Sensor Reading: %d\r\n", r);
            Serial7.printf("Delta Sensor Reading: %d\r\n", curr_delta_dist);

            // Also known as the steer value.
            int pid_result = PID(curr_delta_dist, hist_error, elapsed_time, old_error); 

            int new_l_spd_motor = l_spd_motor + pid_result;
            int new_r_spd_motor = r_spd_motor - pid_result;

            if (new_l_spd_motor > UPPER_MOTOR_LIMIT)
            {
                new_l_spd_motor = UPPER_MOTOR_LIMIT;
            }
            else if (new_l_spd_motor < 0)
            {
                new_l_spd_motor = 0;
            }

            if (new_r_spd_motor > UPPER_MOTOR_LIMIT)
            {
                new_r_spd_motor = UPPER_MOTOR_LIMIT;
            }
            else if (new_r_spd_motor < 0)
            {
                new_r_spd_motor = 0;
            }
            
            Serial7.printf("Steer: %d\r\n", pid_result);
            Serial7.printf("L Motor: %d\r\n", new_l_spd_motor);
            Serial7.printf("R Motor: %d\r\n", new_r_spd_motor);

            setMotorL(FORWARDS, new_l_spd_motor);
            setMotorR(FORWARDS, new_r_spd_motor);
        }

        b_l_val = enc_b_l_val();
        b_r_val = enc_b_r_val();
    }

    Serial7.printf("EXITED WHILE\r\n");
    setMotorL(STOP, 0);
    setMotorR(STOP, 0);

    rstAllEncCounters();
}

void MicroMouse::turnRight()
{
    setMotorL(FORWARDS, l_spd_motor);
    setMotorR(BACKWARDS, 0);
    setMotorR(BACKWARDS, r_spd_motor);

    unsigned int l_val = enc_b_l_val();
    unsigned int r_val = enc_a_r_val();

    while (l_val <= turn_ticks || r_val <= turn_ticks)
    {
        l_val = enc_b_l_val();
        r_val = enc_a_r_val();
    }

    setMotorL(STOP, 0);
    setMotorR(STOP, 0);

    rstAllEncCounters();
}

void MicroMouse::turnLeft()
{
    setMotorL(BACKWARDS, l_spd_motor);
    setMotorR(FORWARDS, 0);
    setMotorR(FORWARDS, r_spd_motor);

    unsigned int l_val = enc_a_l_val();
    unsigned int r_val = enc_b_r_val();

    while (r_val <= turn_ticks || l_val <= turn_ticks)
    {
        l_val = enc_a_l_val();
        r_val = enc_b_r_val();
    }

    setMotorL(STOP, 0);
    setMotorR(STOP, 0);

    rstAllEncCounters();
}

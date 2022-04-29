#include <Arduino.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"

namespace
{
    constexpr bool DEBUG_MODE = 0;
}

MicroMouse::MicroMouse(const unsigned int& enc_a_l, const unsigned int& enc_b_l,
    const unsigned int& enc_a_r, const unsigned int& enc_b_r)
{
    // Initialize all variables necessary for micromouse operations.
    enc_a_l_count = enc_a_l;
    enc_b_l_count = enc_b_l;
    enc_a_r_count = enc_a_r;
    enc_b_r_count = enc_b_r;
}

void MicroMouse::initConnections()
{
    // Setting baud rate.
    Serial.begin(9600);

    // Documentations goes here.
    pinMode(EMIT_L_PIN, OUTPUT);
    pinMode(EMIT_R_PIN, OUTPUT);
    pinMode(EMIT_FL_PIN, OUTPUT);
    pinMode(EMIT_FR_PIN, OUTPUT);

    // Documentations goes here.
    pinMode(RECEIVER_L_PIN, INPUT);
    pinMode(RECEIVER_R_PIN, INPUT);
    pinMode(RECEIVER_FL_PIN, INPUT);
    pinMode(RECEIVER_FR_PIN, INPUT);

    // Documentations goes here.
    pinMode(M1_BACK_PIN, OUTPUT);
    pinMode(M1_FWD_PIN, OUTPUT);
    pinMode(M1_SPD_PIN, OUTPUT);
    pinMode(M1_ENC_A_PIN, INPUT); //check if input pullup
    pinMode(M1_ENC_B_PIN, INPUT);

    // Documentations goes here.
    pinMode(M2_BACK_PIN, OUTPUT);
    pinMode(M2_FWD_PIN, OUTPUT);
    pinMode(M2_SPD_PIN, OUTPUT);
    pinMode(M2_ENC_A_PIN, INPUT); //check if input pullup
    pinMode(M2_ENC_B_PIN, INPUT);

    // Documentations goes here.
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

void MicroMouse::enc_a_l_intr_handler()
{
    ++enc_a_l_count;

    if (DEBUG_MODE)
    {
        Serial.print("ENCAL: ");
        Serial.println(enc_a_l_count);
    }

}

void MicroMouse::enc_b_l_intr_handler()
{
    ++enc_b_l_count;

    if (DEBUG_MODE)
    {
        Serial.print("ENCBL: ");
        Serial.println(enc_b_l_count);
    }
    
}

void MicroMouse::enc_a_r_intr_handler()
{
    ++enc_a_r_count;

    if (DEBUG_MODE)
    {
        Serial.print("ENCAR: ");
        Serial.println(enc_a_r_count);
    }
    
}

void MicroMouse::enc_b_r_intr_handler()
{
    ++enc_b_r_count;

    if (DEBUG_MODE)
    {
        Serial.print("ENCBR: ");
        Serial.println(enc_b_r_count);
    }
    
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
			digitalWrite(M2_FWD_PIN, LOW);
			digitalWrite(M2_BACK_PIN, HIGH);
			analogWrite(M2_SPD_PIN, mspeed);
			break;
		case Direction::BACKWARDS:
			digitalWrite(M2_FWD_PIN, HIGH);
			digitalWrite(M2_BACK_PIN, LOW);
			analogWrite(M2_SPD_PIN, mspeed);
			break;
		case Direction::STOP:
			digitalWrite(M2_FWD_PIN, LOW);
			digitalWrite(M2_BACK_PIN, LOW);
			analogWrite(M2_SPD_PIN, 0);
			break;
	}
}

void MicroMouse::setMotorR(const Direction& dir, const int& mspeed)
{
    switch(dir)
	{
		case Direction::FORWARDS:
			digitalWrite(M1_BACK_PIN, LOW);
			digitalWrite(M1_FWD_PIN, HIGH);
			analogWrite(M1_SPD_PIN, mspeed);
			break;
		case Direction::BACKWARDS:
			digitalWrite(M1_FWD_PIN, LOW);
			digitalWrite(M1_BACK_PIN, HIGH);
			analogWrite(M1_SPD_PIN, mspeed);
			break;
		case Direction::STOP:
			analogWrite(M1_SPD_PIN, 0);
			digitalWrite(M1_FWD_PIN, LOW);
			digitalWrite(M1_BACK_PIN, LOW);
			break;
	}
}

void MicroMouse::setMotorLPulseDir(const Direction& dir, const int& mspeed)
{
    switch(dir)
    {
        case Direction::FORWARDS:
            digitalWrite(M2_FWD_PIN, LOW);
            digitalWrite(M2_SPD_PIN, HIGH);
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
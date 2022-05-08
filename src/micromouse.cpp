#include <Arduino.h>
#include <PID_v1.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"

#define DEBUG 1
#define DEBUG_HANDLER 0
// namespace
// {
//     // Turns on or off debug mode for printing to serial monitor.
//     constexpr bool DEBUG_MODE = 1;
// }

// double r_ip, r_op, r_setp;
// PID t{&r_ip, &r_op, &r_setp, 16.0, 1.0, 4.0, DIRECT};

// By-product of using static member variables. Could this be implemented better? Check note in header file.
// For now, it works fine as long as we do not instanitate another MicroMouse object.
unsigned int MicroMouse::enc_a_l_count = 0;
unsigned int MicroMouse::enc_b_l_count = 0;
unsigned int MicroMouse::enc_a_r_count = 0;
unsigned int MicroMouse::enc_b_r_count = 0;

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

void MicroMouse::enc_a_l_intr_handler()
{
    ++enc_a_l_count;

    #ifndef DEBUG_HANDLER
    Serial.printf("ENCAL: %d\n", enc_a_l_count);
    #endif

    // if (DEBUG_MODE)
    // {
    //     // Serial.print("ENCAL: ");
    //     // Serial.println(enc_a_l_count);
    // }

}

void MicroMouse::enc_b_l_intr_handler()
{
    ++enc_b_l_count;

    #ifndef DEBUG_HANDLER
    Serial.printf("ENCBL: %d\n", enc_b_l_count);
    #endif
    // if (DEBUG_MODE)
    // {
    //     // Serial.print("ENCBL: ");
    //     // Serial.println(enc_b_l_count);
    // }
    
}

void MicroMouse::enc_a_r_intr_handler()
{
    ++enc_a_r_count;

    #ifndef DEBUG_HANDLER
    Serial.printf("ENCAR: %d\n", enc_a_r_count);
    #endif
    // if (DEBUG_MODE)
    // {
    //     // Serial.print("ENCAR: ");
    //     // Serial.println(enc_a_r_count);
    // }
    
}

void MicroMouse::enc_b_r_intr_handler()
{
    ++enc_b_r_count;

    #ifndef DEBUG_HANDLER
    Serial.printf("ENCBR: %d\n", enc_b_r_count);
    #endif

    // if (DEBUG_MODE)
    // {
    //     // Serial.print("ENCBR: ");
    //     // Serial.println(enc_b_r_count);
    // }
    
}

int MicroMouse::getDistL()
{
    int dist;

    // Turns on the emitter, wait for some time, read the numbers reported and store.
    // Turns off the emitter after storing.
    digitalWrite(EMIT_L_PIN, HIGH);
    // analogWrite(EMIT_L_PIN, 220);
    delay(EMITTER_ON_TIME);
    dist = analogRead(RECEIVER_L_PIN);
    digitalWrite(EMIT_L_PIN, LOW);
    // analogWrite(EMIT_L_PIN, 0);

    return dist;
}

int MicroMouse::getDistR()
{
    int dist;

    // Turns on the emitter, wait for some time, read the numbers reported and store.
    // Turns off the emitter after storing.
    digitalWrite(EMIT_R_PIN, HIGH);
    // analogWrite(EMIT_R_PIN, 240);
    delay(EMITTER_ON_TIME);
    dist = analogRead(RECEIVER_R_PIN);
    digitalWrite(EMIT_R_PIN, LOW);
    // analogWrite(EMIT_R_PIN, 0);

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

void MicroMouse::goForward(const int& blocks)
{
    const unsigned int one_cell = 197;
    const unsigned int ticks_to_move = one_cell * blocks;
    int oldErrorP = 0, errorP = 0, errorD = 0;
    
    
    rstAllEncCounters();
    // ip = enc_a_l_count - enc_a_r_count;
    // op = 0.0;
    // setp = 0.0;
    // PID t{&ip, &op, &setp, 16.0, 1.0, 4.0, DIRECT};
    // t.SetMode(AUTOMATIC);
    // t.SetSampleTime(5);
    setMotorL(FORWARDS, 106);
    setMotorR(FORWARDS, 100);
    int totalError = 0;
    const int P = 2;
    const int D = 1;

    while (enc_a_l_val() <= ticks_to_move || enc_b_l_val() <= ticks_to_move)
    {
        oldErrorP = errorP;
        errorP = enc_a_r_count - enc_a_l_count;
        errorD = errorP - oldErrorP;
        totalError = P *errorP + D* errorD;
        if(totalError > 30) {
            totalError = 30;
        } else if (totalError < -30) {
            totalError = -30;
        }
        setMotorL(FORWARDS, 106 - totalError);
        // setMotorR(FORWARDS, 100 + totalError);

        Serial.println(totalError);
        #ifdef DEBUG
        Serial.println("Moving!");
        Serial.printf("Left encoder: %d\n", enc_a_l_val());
        Serial.printf("Right encoder: %d\n", enc_a_r_val());
        #endif
    }

    setMotorL(STOP, 0);
    setMotorR(STOP, 0);
        
    
    // }
//     Serial.println(enc_a_l_val());
//     Serial.println(enc_a_r_val());
//     rstAllEncCounters();
//     setMotorL(STOP, 0);
//     setMotorR(STOP, 0);
    
}
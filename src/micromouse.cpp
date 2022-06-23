#include <Arduino.h>
#include <array>
#include <PID_v1.h>
#include "directions.hpp"
#include "micromouse.hpp"
// #include "testhelper.hpp"
#include "pins.hpp"

#define DEBUG 1

// By-product of using static member variables. Could this be implemented better? Check note in header file.
// For now, it works fine as long as we do not instanitate another MicroMouse object.
volatile unsigned int MicroMouse::enc_backwards_l_count = 0;
volatile unsigned int MicroMouse::enc_forwards_l_count = 0;
volatile unsigned int MicroMouse::enc_backwards_r_count = 0;
volatile unsigned int MicroMouse::enc_forwards_r_count = 0;
int MicroMouse::center = 0;

const unsigned int ticks_to_move = 160;
// 180 appears to be 85 ticks?
const unsigned int turn_ticks = 45;

const float k_p_enc = 0.5f;
// const float k_i_enc = 0.0f;
const float k_d_enc = 0.01f;

const float k_p_ir = 0.08f;
// const float k_i_ir = 0.0f;
const float k_d_ir = 0.005f;

const int l_spd_motor = 125;
const int r_spd_motor = 125;

// const int right_side_wall_threshold = 252;
// const int left_side_wall_threshold = 62;
// const int front_wall_threshold_l = 501;
// const int front_wall_threshold_r = 410;
const int right_side_wall_threshold = 190;
const int left_side_wall_threshold = 80;
const int front_wall_threshold_l = 340;
const int front_wall_threshold_r = 340;

void shiftDirection(short& x, short& y, const Direction& dir, const short& amt)
{
    switch (dir)
    {
        case Direction::FORWARDS:
            y -= amt;
            break;
        case Direction::RIGHT:
            x += amt;
            break;
        case Direction::BACKWARDS:
            y += amt;
            break;
        case Direction::LEFT:
            x -= amt;
            break;
        default:
            break;
    }
}

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
    // Serial7.printf("Center: %d\r\n", center);
    // Serial7.printf("Delta: %d\r\n", sensor_delta);

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
}

void MicroMouse::enc_backwards_r_intr_handler()
{
    ++enc_backwards_r_count;
}

void MicroMouse::enc_forwards_r_intr_handler()
{
    ++enc_forwards_r_count;
    // Serial.printf("%d\r\n", encfor);
}

int MicroMouse::getXpos()
{
    return x_pos;
}

int MicroMouse::getYpos()
{
    return y_pos;
}

Direction MicroMouse::getDir()
{
    return dir;
}

void MicroMouse::setXpos(const int& x)
{
    x_pos = x;
}

void MicroMouse::setYpos(const int& y)
{
    y_pos = y;
}

void MicroMouse::setDir(const Direction& direct)
{
    dir = direct;
}

Walls MicroMouse::detectWalls()
{
    Dists frontDist = getDistFrontRL();
    Dists sideDist = getDistRL();

    return Walls{.l = sideDist.l >= left_side_wall_threshold, 
                 .f = frontDist.r >= front_wall_threshold_r && frontDist.l >= front_wall_threshold_l,
                 .r = sideDist.r >= right_side_wall_threshold};
}

// bool MicroMouse::detectFrontWall() 
// {
//     Dists d = getDistFrontRL();

//     if (d.l >= front_wall_threshold_l && d.r >= front_wall_threshold_r) 
//     {
//         setMotorL(Direction::STOP, 0);
//         setMotorR(Direction::STOP, 0);
//         return true;
//     }

//     return false;
// }

// Walls MicroMouse::detectSideWalls() 
// {
//     Dists d = getDistRL();

//     return Walls {
//             .r = d.r <= right_side_wall_threshold,
//             .l = d.l <= left_side_wall_threshold,
//     };
// }

// std::array<bool, 3> MicroMouse::getWalls()
// {
//     // return getMouseWalls(x_pos, y_pos, dir);
    
//     int left = getDistL();
//     int right = getDistR();
//     int middle = getDistFL();

//     //  = getDistFrontRL();
    
//     bool l =  left > left_side_wall_threshold;
//     bool r = right > right_side_wall_threshold;
//     bool f = middle > front_wall_threshold_l;

//     // if(d.l > front_wall_threshold_l && d.r > front_wall_threshold_r) 
//     // {
//     //     f = true;
//     // }
//     std::array<bool, 3> walls{l, f, r};
    
//     // Serial7.printf("%d, %d, %d\r\n", l, f, r);
//     return walls;
// }

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

// Could have used the getDistL() and getDistR() functions but that would introduce unnecessary delay by 
// calling the functions twice since both functions have a delay of EMITTER_ON_TIME.
Dists MicroMouse::getDistRL()
{
    digitalWrite(EMIT_L_PIN, HIGH);
    digitalWrite(EMIT_R_PIN, HIGH);
    delay(EMITTER_ON_TIME);

    _d.r = analogRead(RECEIVER_R_PIN);
    _d.l = analogRead(RECEIVER_L_PIN);

    digitalWrite(EMIT_L_PIN, LOW);
    digitalWrite(EMIT_R_PIN, LOW);

    return _d;
}

Dists MicroMouse::getDistFrontRL()
{
    digitalWrite(EMIT_FL_PIN, HIGH);
    digitalWrite(EMIT_FR_PIN, HIGH);
    delay(EMITTER_ON_TIME);

    _d.r = analogRead(RECEIVER_FR_PIN);
    _d.l = analogRead(RECEIVER_FL_PIN);

    digitalWrite(EMIT_FL_PIN, LOW);
    digitalWrite(EMIT_FR_PIN, LOW);

    return _d;
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
        default:
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
        default:
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

// unsigned int MicroMouse::enc_backwards_l_val()
// {
//     return enc_backwards_l_count;
// }

// unsigned int MicroMouse::enc_forwards_l_val()
// {
//     return enc_forwards_l_count;
// }

// unsigned int MicroMouse::enc_backwards_r_val()
// {
//     return enc_backwards_r_count;
// }

// unsigned int MicroMouse::enc_forwards_r_val()
// {
//     return enc_forwards_r_count;
// }

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
    rstAllEncCounters();

    setMotorL(Direction::FORWARDS, l_spd_motor);
    setMotorR(Direction::FORWARDS, 0);
    setMotorR(Direction::FORWARDS, r_spd_motor);
    
    // To account for momentum, we need to stop the motors earlier.
    const int offset = 10;

    const unsigned int to_move = (ticks_to_move * blocks) - offset;

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

        proposed_l_spd += pid_ir_result;
        proposed_r_spd -= pid_ir_result;

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

        setMotorL(Direction::FORWARDS, proposed_l_spd);
        setMotorR(Direction::FORWARDS, proposed_r_spd);

    }
    
    rstAllEncCounters();

    setMotorL(Direction::BACKWARDS, 100);
    setMotorR(Direction::BACKWARDS, 100);

    while (enc_forwards_l_count <= offset && enc_forwards_r_count <= offset);

    setMotorL(Direction::STOP, 0);
    setMotorR(Direction::STOP, 0);

    rstAllEncCounters();
}

void MicroMouse::turnRight(const int& blocks)
{
    rstAllEncCounters();

    setMotorL(Direction::FORWARDS, l_spd_motor);
    setMotorR(Direction::BACKWARDS, 0);
    setMotorR(Direction::BACKWARDS, r_spd_motor);

    const int offset = 5;
    const unsigned int to_move = (turn_ticks * blocks) - offset;

    while (enc_forwards_l_count <= to_move || enc_backwards_r_count <= to_move);

    rstAllEncCounters();

    // Why is this needed?
    setMotorL(Direction::BACKWARDS, 100);
    setMotorR(Direction::BACKWARDS, 100);

    while (enc_forwards_l_count <= offset || enc_backwards_r_count <= offset);

    setMotorL(Direction::STOP, 0);
    setMotorR(Direction::STOP, 0);

    rstAllEncCounters();
}

void MicroMouse::turnLeft(const int& blocks)
{
    rstAllEncCounters();

    setMotorL(Direction::BACKWARDS, l_spd_motor);
    setMotorR(Direction::FORWARDS, 0);
    setMotorR(Direction::FORWARDS, r_spd_motor);

    const int offset = 5;
    const unsigned int to_move = (turn_ticks * blocks) - offset;

    while (enc_backwards_l_count <= to_move || enc_forwards_r_count <= to_move); 

    rstAllEncCounters();

    // Why is this needed?
    setMotorL(Direction::BACKWARDS, 100);
    setMotorR(Direction::BACKWARDS, 100);

    while (enc_backwards_l_count <= offset || enc_forwards_r_count <= offset);

    setMotorL(Direction::STOP, 0);
    setMotorR(Direction::STOP, 0);

    rstAllEncCounters();
}
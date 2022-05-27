#ifndef micromouse_hpp
#define micromouse_hpp

#include "directions.hpp"

class MicroMouse
{
public:
    // BEGIN INITIALIZATION FUNCTIONS

    // The job of this function is to initialize all the interal connections
    // from the teensy to all other components in the micromouse.
    void initConnections();
	
    // During interrupts, we know the encoder ticked hence we need to increment
    // our counters.
    void attachInterrupts();

    // During setup phase of the micromouse, we find the center of the mouse relative to
    // the right and left sensors by finding the difference between them.
    void findCenter();

    // END INITIALIZATION FUNCTIONS

	
	// BEGIN MOVEMENT FUNCTIONS

    // Makes the micromouse move forward.
    void goForward(unsigned int& curr_time, const int& blocks);

    // END MOVEMENT FUNCTIONS


    // BEGIN DISTANCE FUNCTIONS

    // Returns the distance the left emitter is receiving.
    int getDistL();

    // Returns the distance the right emitter is receiving.
    int getDistR();
    
    // Returns the distance the front right emitter is receiving.
    int getDistFR();

    // Returns the distance the front left emitter is receiving.
    int getDistFL();

    // Returns the current recorded center of the micromouse.
    int getCenter();

    // END DISTANCE FUNCTIONS


    // BEGIN PID FUNCTION

    // The PID function that calculates the changes for our motors.
    int PID(const int& sensor_data, int& historal_err, const unsigned int& elapsed_time, int& prev_error);

    // END PID FUNCTION


    // BEGIN SET MOTOR FUNCTIONS

    // Sets the left motor with a direction and speed.
    void setMotorL(const Direction& dir, const int& mspeed);

    // Sets the right motor with a direction and speed.
    void setMotorR(const Direction& dir, const int& mspeed);

    // Documentation goes here.
    void setMotorLPulseDir(const Direction& dir, const int& mspeed);

    // Documentation goes here.
    void setMotorRPulseDir(const Direction& dir, const int& mspeed);

    // END SET MOTOR FUNCTIONS


    // BEGIN TURN FUNCTIONS

    // Makes the micromouse turn right.
    void turnRight();

    // Makes the micromouse turn left.
    void turnLeft();

    // END TURN FUNCTIONS

	/*
	We have pins A and B on the encoder to determine whether the encoder is going forward or backwards.
	If in the serial monitor we see ENCA first, we know that we are moving backwards. If
	in the serial monitor we see ENCB first, we know that we are moving forwards.
	*/

    // BEGIN GET ENCODER COUNTER FUNCTIONS

    // Returns the current tick count of the left enc_a.
    unsigned int enc_a_l_val();

    // Returns the current tick count of the left enc_b.
    unsigned int enc_b_l_val();

    // Returns the current tick count of the right enc_a.
    unsigned int enc_a_r_val();

    // Returns the current tick count of the right enc_b.
    unsigned int enc_b_r_val();

    // END GET ENCODER COUNTER FUNCTIONS


    // BEGIN RESET COUNTER FUNCTIONS

    // Resets the left counter for enc_a.
    void rst_enc_a_l_counter();

    // Resets the left counter for enc_b.
    void rst_enc_b_l_counter();

    // Resets the right counter for enc_a.
    void rst_enc_a_r_counter();

    // Resets the right counter for enc_b.
    void rst_enc_b_r_counter();

    // Resets all encoder counters.
    void rstAllEncCounters();

    // END RESET COUNTER FUNCTIONS


    // BEGIN DEBUG FUNCTIONS

    // Turns all the emitters on.
    void turnAllEmittersOn();

    // Turns all the emitters off.
    void turnAllEmittersOff();

    // Turns all LEDs on.
    void turnAllLedOn();

    // Turns all LEDs off.
    void turnAllLedOff();

    // Turns on buzzer.
    void setBuzzerOn();

    // Turns off buzzer.
    void setBuzzerOff();

    //END DEBUG FUNCTIONS

    /*
    NOTE: 
    The functions and variables that are labeled static need to be treated carefully.
    This implementation only works because we are not instantiating more than one MicroMouse
    object in main.cpp. If we were to create more than one MicroMouse object, changes WOULD have
    to be made.

    REASON:
    ISRs (InterruptServiceRoutines), specifically the function attachInterrupt from Arduino,
    has a parameter that takes in a function. The function has to be global and not in a class.

    For more information (and a solution for a workaround):
    https://arduino.stackexchange.com/questions/73287/interrupts-inside-a-class-attaching-to-the-function-of-the-class
    https://www.gammon.com.au/forum/?id=12983

    The current implementation DOES NOT use this workaround but instead uses statics.
    */

    // The main job of this function is to increment our left enc_a and enc_b counters
    // to be able to tell how many ticks the encoder has moved.
    static void enc_backwards_l_intr_handler();
    static void enc_forwards_l_intr_handler();

    // The main job of this function is to increment our right enc_a and enc_b counters
    // to be able to tell how many ticks the encoder has moved.
    static void enc_backwards_r_intr_handler();
    static void enc_forwards_r_intr_handler();

private:
    /* 
    These variables are for the left motor encoder. 
    We have two distinctions: enc_a and enc_b. This is because
    the combination of these two allow us to tell if the encoder is
    moving forward or backwards.
    */
    static volatile unsigned int enc_backwards_l_count;
    static volatile unsigned int enc_forwards_l_count;

    /* 
    These variables are for the right motor encoder. 
    We have two distinctions: enc_a and enc_b. This is because
    the combination of these two allow us to tell if the encoder is
    moving forward or backwards.
    */
    static volatile unsigned int enc_backwards_r_count;
    static volatile unsigned int enc_forwards_r_count;

    // Stores the sensor reading of the mouse when placed in the center during setup.
    static int center;
};

#endif
#ifndef micromouse_hpp
#define micromouse_hpp

#include "directions.hpp"

struct Dists
{
    int r;
    int l;
};

struct Walls
{
    bool r;
    bool l;
};

void shiftDirection(short& x, short& y, const Direction& dir, const short& amt = 1);

class MicroMouse
{
public:
    MicroMouse(const short& x, const short& y, const Direction& dir);

    // BEGIN INITIALIZATION FUNCTIONS

    // The job of this function is to initialize all the interal connections
    // from the teensy to all other components in the micromouse.
    static void initConnections();
	
    // During interrupts, we know the encoder ticked hence we need to increment
    // our counters.
    static void attachInterrupts();

    // During setup phase of the micromouse, we find the center of the mouse relative to
    // the right and left sensors by finding the difference between them.
    void findCenter();

    // END INITIALIZATION FUNCTIONS

    // BEGIN GET POSTION FUNCTIONS

    int getXpos();

    int getYpos();

    Direction getDir();

	// BEGIN MOVEMENT FUNCTIONS

    // Makes the micromouse move forward.
    virtual void goForward(const int& blocks = 1);

    // END MOVEMENT FUNCTIONS


    // BEGIN TURN FUNCTIONS

    // Makes the micromouse turn right.
    virtual void turnRight(const int& blocks = 1);

    // Makes the micromouse turn left.
    virtual void turnLeft(const int& blocks = 1);

    // END TURN FUNCTIONS


    // BEGIN DISTANCE FUNCTIONS

    // Returns the distance the left emitter is receiving.
    static int getDistL();

    // Returns the distance the right emitter is receiving.
    static int getDistR();
    
    // Returns the distance the front right emitter is receiving.
    static int getDistFR();

    // Returns the distance the front left emitter is receiving.
    static int getDistFL();

    // Returns a struct containing the distances read from the right and left emitters.
    Dists getDistRL();

    Dists getDistFrontRL();

    // END DISTANCE FUNCTIONS


    // BEGIN CHECK WALLS
    bool detectFrontWall();

    Walls detectSideWalls();

    // END CHECK WALLS

    // BEGIN SET MOTOR FUNCTIONS

    // Sets the left motor with a direction and speed.
    static void setMotorL(const Direction& dir, const int& mspeed);

    // Sets the right motor with a direction and speed.
    static void setMotorR(const Direction& dir, const int& mspeed);

    // Documentation goes here.
    void setMotorLPulseDir(const Direction& dir, const int& mspeed);

    // Documentation goes here.
    void setMotorRPulseDir(const Direction& dir, const int& mspeed);

    // END SET MOTOR FUNCTIONS

	/*
	We have pins A and B on the encoder to determine whether the encoder is going forward or backwards.
	If in the serial monitor we see ENCA first, we know that we are moving backwards. If
	in the serial monitor we see ENCB first, we know that we are moving forwards.
	*/

    // BEGIN GET ENCODER COUNTER FUNCTIONS

    // Returns the current tick count of the left enc_a.
    unsigned int enc_backwards_l_val();

    // Returns the current tick count of the left enc_b.
    unsigned int enc_forwards_l_val();

    // Returns the current tick count of the right enc_a.
    unsigned int enc_backwards_r_val();

    // Returns the current tick count of the right enc_b.
    unsigned int enc_forwards_r_val();

    // END GET ENCODER COUNTER FUNCTIONS


    // BEGIN RESET COUNTER FUNCTIONS

    // Resets the left counter for enc_a.
    void rst_enc_backwards_l_counter();

    // Resets the left counter for enc_b.
    void rst_enc_forwards_l_counter();

    // Resets the right counter for enc_a.
    void rst_enc_backwards_r_counter();

    // Resets the right counter for enc_b.
    void rst_enc_forwards_r_counter();

    // Resets all encoder counters.
    void rstAllEncCounters();

    // END RESET COUNTER FUNCTIONS


    // BEGIN DEBUG FUNCTIONS

    // Turns all the emitters on.
    static void turnAllEmittersOn();

    // Turns all the emitters off.
    static void turnAllEmittersOff();

    // Turns all LEDs on.
    static void turnAllLedOn();

    // Turns all LEDs off.
    static void turnAllLedOff();

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

protected:
    short x_pos, y_pos;
    Direction dir;

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

    Dists _d;


    // BEGIN PID FUNCTION

    // The PID functions that calculates the changes for our motors.
    int PID_IR(int& historal_err, int& prev_error);
    int PID_enc(int& historal_err, int& prev_error);

    // END PID FUNCTION

};

#endif
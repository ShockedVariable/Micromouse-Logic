#include <Arduino.h>
#include <PID_v1.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"

MicroMouse mm;


void setup() 
{
	mm.initConnections();
	mm.attachInterrupts();

	delay(2000);
}


//no resistance, battery full charge
//160 measures about 6v
//150 measure about 6v
//140 measure about 5.9v

//reccomend 

//Ran with motor limit 140, 
//with motors maxed out, all emitters on, all leds on, (buzzer off), no resistance at wheels. it drew about 0.9A.



// int last_spd;

void loop()
{
	// mm.setMotorL(FORWARDS, 90);
	// mm.setMotorR(FORWARDS, 90);
	// mm.rstAllEncCounters();

	mm.setMotorL(FORWARDS, 100);
	mm.setMotorR(FORWARDS, 100);

	mm.goForward(2);
	// while(true) {

	// }
	
	// if (mm.enc_a_l_val() == 197 || mm.enc_b_l_val() == 197)
	// {
	// 	mm.setMotorL(STOP, 0);
	// 	mm.setMotorR(STOP, 0);

	// 	mm.rstAllEncCounters();

	// }
}


//cell = 18 cm
//cell gap = 1.2 cm
//diameter wheel to wheel 9.1 cm
//wheel diameter is 2.8 cm
//90 ticks for 1 revolution the wheel
//
//approx 197 ticks for to move one cell
//approx 73 ticks to turn 90 degrees

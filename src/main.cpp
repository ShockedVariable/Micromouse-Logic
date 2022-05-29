#include <Arduino.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"

MicroMouse mm(0, 0, Direction::FORWARDS);

void setup() 
{
	mm.initConnections();
	mm.attachInterrupts();

	mm.findCenter();

	delay(2000);
	
	// Serial7 is default for bluetooth module.
	Serial7.begin(9600);
}

void loop()
{	
	while (mm.getDistFR() < 900);

	delay(1000);
	mm.goForward(25);
	// mm.turnRight(25);
	delay(2500);
	
}

//no resistance, battery full charge
//160 measures about 6v
//150 measure about 6v
//140 measure about 5.9v

//reccomend 

//Ran with motor limit 140, 
//with motors maxed out, all emitters on, all leds on, (buzzer off), no resistance at wheels. it drew about 0.9A.


//cell = 18 cm
//cell gap = 1.2 cm
//diameter wheel to wheel 9.1 cm
//wheel diameter is 2.8 cm
//90 ticks for 1 revolution the wheel
//
//approx 197 ticks for to move one cell
//approx 73 ticks to turn 90 degrees


// calibration tip: for the first couple of seconds
// let the mouse remember what is the "center" as reference point 
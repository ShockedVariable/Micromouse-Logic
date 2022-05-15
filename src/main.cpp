#include <Arduino.h>
#include <PID_v1.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"
#include "TeensyTimerTool.h"

MicroMouse mm;
TeensyTimerTool::PeriodicTimer t1(TeensyTimerTool::TCK64);
unsigned int curr_time = 0;

void inc()
{
	++curr_time;
}

void setup() 
{
	mm.initConnections();
	mm.attachInterrupts();

	mm.findCenter();

	delay(2000);
	// Serial7 is default for bluetooth module.
	Serial7.begin(9600);
	delay(2500);

	Serial7.printf("Center: %d\r\n", mm.getCenter());

	t1.begin(inc, 100); // 100 microseconds
}

void loop()
{
	mm.goForward(curr_time, 5);
	delay(1000);
	mm.turnLeft();
	delay(1000);
	mm.turnRight();

	while (true);
	// unsigned short left_sensor = mm.getDistL();
	// unsigned short right_sensor = mm.getDistR();
	// unsigned short f_left_sensor = mm.getDistFL();
	// unsigned short f_right_sensor = mm.getDistFR();
	// digitalWrite(EMIT_L_PIN, HIGH);
	// digitalWrite(EMIT_R_PIN, HIGH);
	// digitalWrite(EMIT_FR_PIN, HIGH);
	// digitalWrite(EMIT_FL_PIN, HIGH);

	// if (left_sensor > -1)
	// {
	// 	// Serial.printf("Left Sensor: %d\n", left_sensor);
	// 	Serial7.printf("Left Sensor: %d\n", left_sensor);
	// }
	
	// if (right_sensor > -1)
	// {
	// 	// Serial.printf("Right Sensor: %d\n", right_sensor);
	// 	Serial7.printf("Right Sensor: %d\n", right_sensor);
	// }

	// if (f_left_sensor > 50)
	// {
	// 	Serial.printf("Front Left Sensor: %d\n", f_left_sensor);
	// }
	
	// if (f_right_sensor > -1)
	// {
	// 	Serial.printf("Right Sensor: %d\n", f_right_sensor);
	// 	Serial7.printf("Right Sensor: %d\n", f_right_sensor);
	// }
	

	
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
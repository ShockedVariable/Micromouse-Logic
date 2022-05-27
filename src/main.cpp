#include <Arduino.h>
#include "directions.hpp"
#include "micromouse.hpp"
#include "pins.hpp"

MicroMouse mm;

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

	mm.goForward(3);
	// delay(1000);
	// mm.turnLeft();
	// delay(1000);
	// mm.turnRight();
	// Serial.printf("Left Encoder: %d\r\n", mm.enc_b_l_val());
	// Serial.printf("Right Encoder: %d\r\n", mm.enc_b_r_val());

	// delay(1000);
	while (true);
	// unsigned short left_sensor = mm.getDistL();
	// unsigned short right_sensor = mm.getDistR();
	// unsigned short f_left_sensor = mm.getDistFL();
	// unsigned short f_right_sensor = mm.getDistFR();

	// if (left_sensor > -1)
	// {
	// 	Serial.printf("Left Sensor: %d\n", left_sensor);
	// 	// Serial7.printf("Left Sensor: %d\n", left_sensor);
	// }
	
	// if (right_sensor > -1)
	// {
	// 	Serial.printf("Right Sensor: %d\n", right_sensor);
	// 	// Serial7.printf("Right Sensor: %d\n", right_sensor);
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

//Clock Crane V0.1 by Sean J. Miller 2018.  Controls a robotic crane disguised as a decorative wall clock.
//To compile, do the following:
//1.  Make sure you have installed the HAL package from the Matrix.Creator repository.
//				https://matrix-io.github.io/matrix-documentation/matrix-hal/getting-started/installation/
//2.  cd ~/matrix-creator-hal/demos.
//3.  paste this code there in a file named cc.cpp
//4.  use the following command to compile:
//5.		g++ -o cc -I /usr/include/matrix_hal -O /usr/lib/libmatrix_creator_hal.so cc.cpp -lpthread
//6.  execute the file by typing this:  ./cc
//Hardware:  Matrix.Creator, Raspberry Pi 3, Dimension Engineering Sabertooth 2x32.  See #define code below for hookups.

#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>
#include <string>
#include "../cpp/driver/fw_data.h"
#include "../cpp/driver/imu_data.h"
#include "../cpp/driver/imu_sensor.h"
#include "../cpp/driver/matrixio_bus.h"
#include "../cpp/driver/everloop.h"
#include "../cpp/driver/everloop_image.h"
#include "../cpp/driver/gpio_control.h"

namespace hal = matrix_hal;

#define TEST true  //used to flip between a test mode that runs a motor test routine and its production program.
#define INPUT 0 //GPIO declaration of input pin
#define OUTPUT 1 //GPIO declaration of output pin
#define UP 0 //GPIO setting for voltage to flag the motor controller to go up.
#define DOWN 1 //GPIO setting for voltage to flag the motor controller to go down.
#define OFF 0 //GPIO setting to set 0V.
#define ON 1  //GPIO setting to set 3.3V out.
#define CLOCKWISE 0 //GPIO setting for voltage to flag the motor controller to turn the rotation motor clockwise.
#define COUNTER_CLOCKWISE 1 //GPIO setting for voltage to flag the motor controller to turn the rotation counter-clockwise.
#define BOOM_MOTOR 0 //GPIO pin 0 is the output pin to flag speed of the boom on the motor controller.  GPIO 0 hooks to S1 on the motor controller.
#define BOOM_DIRECTION 1 //GPIO pin 1 is the output pin to flag direction of the boom (up or down).  GPIO 1 hooks to A1 on the motor controller.
#define ROTATION_MOTOR 2 //GPIO pin 2 is the output pin to flag speed of the rotation.  GPIO 2 hooks to S2 on the motor controller.
#define ROTATION_DIRECTION 3 //GPIO pin 3 is the output pin to flag direction of the rotation (Clockwise versus Counter Clockwise).  GPIO 3 hooks to A2 on the motor controller.
#define START_BUTTON 4 //GPIO 4 is an input pin to sense that the user has pressed an external button.  The button is used to start a demo routine until we get voice commands working.
#define CLK_FRQ 200000000 //I really don't know why this is set, but it was in other example code.

hal::MatrixIOBus bus;//variable that represents the matrix in general.
hal::GPIOControl gpio;//variable to reference GPIO pin methods.
hal::IMUData imu_data;//variable that stores sensor data when called.
hal::IMUSensor imu_sensor;//variable to reference the sensors as an object.

bool lights=false; //Global variable to control the everloopThread which makes a light show when desired by a method.
std::string state="idle"; //state is a global variable that tracks what the crane is doing.  This is useful for threads to adjust their activity based on the current state.

void stopAll() {
		//this routine stops all motion by setting all GPIO outputs to 0v
		gpio.SetGPIOValue(BOOM_MOTOR,0);
		gpio.SetGPIOValue(BOOM_DIRECTION,0);
		gpio.SetGPIOValue(ROTATION_MOTOR,0);
		gpio.SetGPIOValue(ROTATION_DIRECTION,0);
}

void buttonHandler(){
	//This routine polls the input pins and stores them in a variable.  In our case, if the GPIO for the start button is found pressed, it changes the state variable to "start crane".
	uint16_t read_data = 0;

	while (true) {
		read_data = gpio.GetGPIOValues();
		if (read_data<=1&&state=="idle") {state="start crane";usleep(200000);}//The usleep command gives time for them to release the switch before polling again.  5 is released		
	usleep(5000);
	}
}

void sensorReader() {
	//This is the thread that refreshes the sensor data available to the threads.
	while (true) { 
		imu_sensor.Read(&imu_data);
		std::system("clear");
		std::cout << "Status=" << state <<"\t\t";
		std::cout << "yaw = " << imu_data.yaw << "°\t\t";
		std::cout << "roll = " << imu_data.roll << "°\t\t";
		std::cout << "pitch = " << imu_data.pitch << "°\t\t";
		std::cout << "lights = " << lights << "°" << std::endl;

		usleep(500000);//pause .5 seconds
	}//continuosly poll for sensor data
}

void raiseBoom() {
	state="raising boom"; lights=true;
	stopAll();//Stop any movement.
	if (imu_data.roll<-5) {//turn on the Boom liner actuator and drive it until it is where we want it.
		gpio.SetGPIOValue(BOOM_MOTOR,ON);//PUT 5V TO THE MOTOR CONTROLLER BOOM MOTOR
		gpio.SetGPIOValue(BOOM_DIRECTION,UP);//put 5V on the pin to tell the direction of the boom motor.
		while (imu_data.roll<-5&&lights) usleep(10000);//Monitor the progress of it going up.
	}
	lights=false;//At this point, it's reached the upright postion, 
	stopAll;//so we can turn off the motor.
}

void turnBoomOut() {
	state="turning boom out"; lights=true;
	stopAll();//Stop any movement.
	if (imu_data.yaw>60) {//turn on the rotation motor and drive it until the yaw is where we want it.
			gpio.SetGPIOValue(ROTATION_MOTOR,ON);//put 3.3V on the motor controller rotation motor
	gpio.SetGPIOValue(ROTATION_DIRECTION,CLOCKWISE);//goes to the analog pin on the motor controller that tells direction
			while (imu_data.yaw>60&&lights) usleep(10000);//Monitor the progress of it going up.
	}
	lights=false;//At this point, it's reached the upright postion,
	stopAll();//so we can turn off the motor.
}

void lowerHook(){
	state="lowering hook";
	//code pending
	usleep(4000000);
}

void raiseHook(){
	state="raising hook";
	//code pending
	usleep(4000000);
}

void turnBoomToCenter() {
	state="turning boom to center"; lights=true;
	stopAll();//Stop any movement
	if (imu_data.yaw<100) {//turn on the rotation motor and drive it until the yaw is where we want it.
		gpio.SetGPIOValue(ROTATION_MOTOR,ON);
		gpio.SetGPIOValue(ROTATION_DIRECTION,COUNTER_CLOCKWISE);
		while (imu_data.yaw<100&&lights) usleep(10000);
	}
	lights=false;
	stopAll();
}

void lowerBoom() {
	state="lowering boom";lights=true;
	stopAll();//Stop any movement.
	if (imu_data.roll>-88) {//turn on the boom linear actuator and drive it until it is where we want it.
		gpio.SetGPIOValue(BOOM_MOTOR,ON);
		gpio.SetGPIOValue(BOOM_DIRECTION,DOWN);
		while (imu_data.roll>-88&&lights) usleep(10000);
	}
	lights=false;
	stopAll;
}

void strobeLights () {
	hal::Everloop everloop;
	hal::EverloopImage image1d(bus.MatrixLeds());
	everloop.Setup(&bus);

	unsigned counter=0;
		while (true) {
		while(!lights) usleep(10000);

		for (hal::LedValue &led : image1d.leds) {
		  led.red = 0;
		  led.green = 0;
		  led.blue = 0;
		  led.white = 0;
		}

		image1d.leds[(counter / 2) % image1d.leds.size()].red = 20;
		image1d.leds[(counter / 7) % image1d.leds.size()].green = 30;
		image1d.leds[(counter / 11) % image1d.leds.size()].blue = 30;
		image1d.leds[image1d.leds.size() - 1 - (counter % image1d.leds.size())].white = 10;
		everloop.Write(&image1d);
		++counter;
		usleep(10000);
	}
}

void setup() {
  //These arrays just give a clean looking way to setup the GPIO pins.
  unsigned char inputPinList[] = {START_BUTTON};//So far, we only have on input.  It's the button used for triggering a test routine.
  unsigned char outputPinList[] = {BOOM_MOTOR, BOOM_DIRECTION, ROTATION_MOTOR, ROTATION_DIRECTION};//outputs to the motor controller for the boom motion.
  
  imu_sensor.Setup(&bus);  //setup the sensors on the Matrix.Creator
  gpio.Setup(&bus);  //setup the GPIO on the Matrix.Creator
  gpio.SetMode(inputPinList, sizeof(inputPinList), INPUT);
  gpio.SetMode(outputPinList, sizeof(outputPinList), OUTPUT);
  gpio.SetGPIOValues(outputPinList, sizeof(outputPinList), 0);
}

void mainLoop() {
	 while (true) {//main thread that loops similar to loop on the Arduino.  However, we have threads going too, which is awesome.
		if (state=="start crane") {
			raiseBoom();
			turnBoomOut();
			lowerHook();
			raiseHook();
			turnBoomToCenter();
			lowerBoom();
			state="idle";
		}
		usleep(200000);
	}
}

void testMode(){
	//This is used if the preprocessor definition (#define) DEBUG is true.
	while (true) {
		raiseBoom();
		usleep(3000000);
		lowerBoom();
		usleep(3000000);
	}
}

int main() {
  if (!bus.Init()) return false;//if the Matrix.Creator doesn't initialize, then terminate the program.

  setup();//call the routine that sets up sensors and gpio.

  std::thread EverloopThread (strobeLights); //start our strobeLight routine in its own thread.  It monitors variables to determine ow it should show lights.
  std::thread ButtonThread (buttonHandler);//thread to monitor the button presses.  
  std::thread SensorDataThread (sensorReader);//thread to check values of all sensors to make imu_data available fresh for all methods concerned.

 if (!TEST) mainLoop(); else testMode();//If you compile with DEBUG true (see #define's), then it will do the testMode routine so you can test, monitor, and tweak the mechanical system.
}
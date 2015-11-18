#include <stdlib.h>
#include <wiringPi.h>

#define TRIG 5

void SonarInit() {
	wiringPiSetup();
}

int SonarGetCM() {
  	pinMode(TRIG, OUTPUT);
	digitalWrite(TRIG, LOW);
	delay(30);
	digitalWrite(TRIG, HIGH);
	delay(50);
	digitalWrite(TRIG, LOW);
 	pinMode(TRIG, INPUT);

	//Wait for echo start
	while(digitalRead(TRIG) == LOW);

	//Wait for echo end
	long startTime = micros();
	while(digitalRead(TRIG) == HIGH);
  	long travelTime = micros() - startTime;

	//Get distance in cm
	int distance = travelTime / 58;
	return distance;
}

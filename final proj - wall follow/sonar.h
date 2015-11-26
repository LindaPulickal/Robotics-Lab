#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>
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


void turnSonar(float angle, FILE *fp) {
	fprintf(fp, "2=%d\n", angle*10/9 +50);
	fflush(fp);

}
void turnSonarCenter(FILE *fp) {
	fprintf(fp, "2=150\n");
	fflush(fp);

}
void turnSonarLeft(FILE *fp) {
	fprintf(fp, "2=240\n");
	fflush(fp);

}
void turnSonarRight(FILE *fp) {
	fprintf(fp, "2=50\n");
	fflush(fp);

}

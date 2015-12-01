#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
//#include "pid.h"
#define DRIVE_ZERO 150
#define DRIVE_WHEEL_DISTANCE_CM 100

class DriveController{

private:
    float linearVelocity;
    float rotationalVelocity;
    FILE* fp;
    //PIDController *pidControl;
    float wallDistance;

    void updateSpeed(){
	float left = DRIVE_ZERO  + 4 +linearVelocity + rotationalVelocity ; //4   
        float right = DRIVE_ZERO  + 3 - linearVelocity + rotationalVelocity; //3
//	std::cout<<"\nleft : "<<left <<"  ::  right : "<<right;
	fprintf(fp, "0=%f\n", left);
        fprintf(fp, "1=%f\n", right);
        fflush(fp);
    }

public:

    DriveController() {
    	linearVelocity = 0;
    	rotationalVelocity = 0;
    	//pidControl = new PIDController(0.5,0,0);
    	wallDistance = 50;
    }

    void setServoBlaster(FILE *file) {
        fp = file;
    }

    void DriveLinear(float velocity){
        linearVelocity = velocity;
        updateSpeed();
    }

    void DriveRotational(float velocity){
        rotationalVelocity = velocity;
        updateSpeed();
    }
	
    void DriveTurn(float theta){
	DriveLinear(0);
	DriveRotational(theta/4);
	delay(600);
	DriveRotational(0);
	}


	void DriveDistancePID(float distanceCM, float turn) {
        DriveRotational(turn);
        DriveLinear(5);
        delay(distanceCM*1000/15);
	delay(180);
        DriveRotational(0);
        DriveLinear(0);
    }

    void DriveDistance(float distanceCM){
	DriveRotational(0);
	DriveLinear(5);
	delay(distanceCM*1000/15);
	DriveLinear(0);
    }
    
    void align90(float angle) {
    	DriveRotational(90-angle);
    }
};

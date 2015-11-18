#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#define DRIVE_ZERO 150
#define DRIVE_WHEEL_DISTANCE_CM 100

class DriveController{

private:
    float linearVelocity;
    float rotationalVelocity;
    float left;
    float right;
    float corrector;
    FILE* fp;

    void ChangeSpeed(){
	if (linearVelocity != 0)
		corrector = 2;
	else
		corrector = 0;
        left = DRIVE_ZERO + 4 + linearVelocity + rotationalVelocity ;   
        right = DRIVE_ZERO  + 3 - linearVelocity + rotationalVelocity;
	//printf("\nLeft:%f",left);
	//printf("\nRight:%f",right);
	//printf("\nRot:%f",rotationalVelocity);
	fflush(stdout);
        fprintf(fp, "0=%f\n", left);
        fprintf(fp, "1=%f\n", right);
        fflush(fp);
    }

public:

    DriveController() {
    	fp = fopen("/dev/servoblaster", "w");
    	if(fp == NULL){
    		printf("Error opening servo blaster file\n");
    		exit(0);
    	}
    	left = DRIVE_ZERO;
    	right = DRIVE_ZERO;
    	linearVelocity = 0;
    	rotationalVelocity = 0;
    }

    ~DriveController() {
        fclose(fp);
    }

    void DriveLinear(float linearVelocity_){
        linearVelocity = linearVelocity_;
        ChangeSpeed();
    }
    void turnSonar(float angle) {
	//delay(00);
	fprintf(fp, "2=%f\n", angle*10/9 +50);
	fflush(fp);
	}

    void DriveRotational(float rotationalVelocity_){
        rotationalVelocity = rotationalVelocity_;
        ChangeSpeed();
    }
		
		void DriveTurn(float theta){
			printf("Angle:%f", theta);
				fflush(stdout);
			DriveLinear(0);
			DriveRotational(theta/2);
			delay(500);
			DriveRotational(0);
		}
		
		void DriveDistance(float distanceCM){
			DriveRotational(0);
			DriveLinear(distanceCM);
			delay(500);
			DriveLinear(0);
			
		}
};

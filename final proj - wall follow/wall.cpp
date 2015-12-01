#include <iostream>
#include<stdlib.h>
#include "sonar.h"
#include <wiringPi.h>
#include "drive.h"
#include "pid.h"
#include "point.h"
#include "A_Star.h"
using namespace std;

PIDController pidControl(0.1,0.05,0);
DriveController driver;
FILE *fp;
float WALL_DISTANCE = 40;
int GRIDSIZE = 30;
bool LAB_A_REACHED = false;
bool LAB_B_REACHED = false;
bool VIRUS_A_FOUND = false;
bool VIRUS_B_FOUND = false;

void initializeControls() {
	fp = fopen("/dev/servoblaster", "w");
	if(fp == NULL){
		printf("Error opening servo blaster file\n");
		exit(0);
	}
	SonarInit();
	driver.setServoBlaster(fp);
}

int moveTo(Point currPoint, float &currAngle, Point wayPoint, int direction){
	float angleRotate = currAngle;
	if(direction == 0) //move down
		angleRotate = 90 - currAngle;
	if(direction == 1) //move left 
		angleRotate = 0 - currAngle;
	if(direction == 2) //move up
		angleRotate = -90  - currAngle;
	if(direction == 3) // move right
		angleRotate =  currAngle - 0;
	driver.DriveTurn(angleRotate); // when anglerotate is negative it is left turn
	currAngle += angleRotate;
	
	float pidOut, distanceLeft, distanceRight;
	for(int i = 0; i<2; i++) {
		turnSonarRight(fp);
		delay(500);
		distanceRight = SonarGetCM();  
		delay(1000);
		turnSonarLeft(fp);
		delay(500);
		distanceLeft = SonarGetCM();  
		//turnSonarCenter(fp);
		if (distanceLeft > 60 && distanceRight >60){
			driver.DriveDistancePID(15,0);
			cout<<"\n no wall"; fflush(stdout);}

		else if(distanceLeft < distanceRight) {
			pidOut = pidControl.getPIDValue(WALL_DISTANCE, distanceLeft);
	cout<<"\nleft pid= "<<pidOut;		fflush(stdout);
			driver.DriveDistancePID(15, pidOut);
		}
		else if(distanceRight < distanceLeft) {
			pidOut = pidControl.getPIDValue(WALL_DISTANCE, distanceRight);		
			driver.DriveDistancePID(15, -pidOut);
	cout<<"\nright pid = "<<pidOut;		fflush(stdout);

		}
		delay(1000);
	}
	//driver.DriveDistance(30);
	//driver.DriveDistance((float)(sqrt(pow((double)(wayPoint.x - currPoint.x), 2) + pow((double)(wayPoint.y - currPoint.y), 2))));
}

int goToGoal(Point currLoc, Point goalLoc, float& angle){
	angle = 90; //assumes that the robot is always facing down when it starts
	string path = pathFind(currLoc.x, currLoc.y , goalLoc.x ,goalLoc.y);
	cout<<"path = "<<path;
	for(char i=0; i<path.length(); i++) {
		int dir = path[i] - '0';
		Point nextLoc(currLoc.x + dx[dir], currLoc.y + dy[dir]);
		moveTo(currLoc, angle, nextLoc, dir);
		currLoc = nextLoc;
	}
	driver.DriveLinear(0);
	driver.DriveRotational(0);
}

void wallFollow() {
	float goalDistance = 30;
	while(1) {
		float currDistance = SonarGetCM();
		float pidOut = pidControl.getPIDValue(WALL_DISTANCE, currDistance);
		cout<<"\nPID : "<<pidOut;
		driver.DriveLinear(5);
		driver.DriveRotational(pidOut);
		delay(200);
	}
}

int getVirus() {
	return 1 ;
}

int main() {
	initializeControls();
	Point goal_possibleVirus1(8,5);
	//driver.DriveDistancePID(30,0);

	Point goal_VirusLocs[3] = {{1,5}, {1,7}, {8,5}};
    Point goal_LabA(5,1);
    Point goal_LabB(10,9);

    Point currLoc(1,1);
    float angle;
    for(int i =0; i<3; i++) {
    	    if (LAB_A_REACHED && LAB_B_REACHED)
    			break;

            goToGoal(currLoc, goal_VirusLocs[i], angle);
            int virusStatus = getVirus();
            currLoc = goal_VirusLocs[i]; 
            driver.align90(angle);
            if (virusStatus == 0) {
                    continue;
            }

            else if (virusStatus == 1) {
            	VIRUS_A_FOUND = true;
            	goToGoal(currLoc, goal_LabA, angle);
            	LAB_A_REACHED = true;
		driver.align90(angle);
            }
			
	else if (virusStatus == 2) {
            	VIRUS_B_FOUND = true;
            	goToGoal(currLoc, goal_LabB, angle);
            	LAB_B_REACHED = true;
		driver.align90(angle);		
            }

    }
    //string a  = pathFind(1,1, 10, 1);cout<<a;
    return 0;
}


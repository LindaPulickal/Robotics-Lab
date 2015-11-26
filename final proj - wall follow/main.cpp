#include <iostream>
#include "sonar.h"
#include <wiringPi.h>
#include "drive.h"
#include "pid.h"
#include "navigation.h"
/*#include <vector>
#include "Util_Map.h"
#include "point.h"
#include "raycaster.h"

#include <math.h>
#include <stdlib.h>     
#include <time.h>       
#include "A_Star.h"
#include "navigation.h"
#include "localize.h"*/

using namespace cv;
using namespace std;

PIDController *pidControl;
DriveController *driver;

Point goal_possibleVirus1(1,5);
Point goal_possibleVirus2;
Point goal_possibleVirus3;
Point goal_blueLab;
Point goal_redLab;

const int GRIDSIZE = 30;
int RobotSize = 50;
int angleRot = 0;
int angle = 90;
int DIS_VARIANCE = 5;
int ANG_VARIANCE = 0;
int DISTANCE = 5;
Point RobotCenter;
File* fp; 

void initializeControls() {
	fp = fopen("/dev/servoblaster", "w");
	if(fp == NULL){
		printf("Error opening servo blaster file\n");
		exit(0);
	}
	sonarInit();
	driver = new DriveController(fp);
	pidControl = new PIDController(0.5,0,0);
}

int moveTo(Point currPoint, float &currAngle, Point wayPoint, int direction){
	float angleRotate = currAngle;
	if(direction == 0)
		angleRotate = 90 - currAngle;
	if(direction == 1)
		angleRotate = 0 - currAngle;
	if(direction == 2)
		angleRotate = -90  - currAngle;
	if(direction == 3)
		angleRotate =  currAngle - 0;
	driver.DriveTurn(angleRotate);
	currAngle += angleRotate;
	driver.DriveDistance(15);
	turnSonar(fp, 180);
	//driver.DriveDistance((float)(sqrt(pow((double)(wayPoint.x - currPoint.x), 2) + pow((double)(wayPoint.y - currPoint.y), 2))));
}

/*
int goToGoal(Point currLoc, Point goalLoc){
	float angle = 90;
	string path = pathFind(currLoc.x, currLoc.y , goalLoc.x ,goalLoc.y);
	for(char i=0; i<path.length(); i++) {
		int dir = path[i] - '0';
		Point nextLoc(currLoc.x + dx[dir]*GRIDSIZE, currLoc.y + dy[dir]*GRIDSIZE);
		moveTo(currLoc, angle, nextLoc, dir);
		currLoc = nextLoc;
	}
}
*/
void wallFollow() {
	float goalDistance = 30;
	for(int i = 0 ; i < 1000; i++) {
		float currDistance = SonarGetCM();
		float pidOut = pidControl.getPIDValue(goalDistance, currDistance);
		driver.DriveLinear(5);
		driver.DriveRotational(pidOut);
	}
}

int main(){
	initializeControls();
	turnSonar(fp, 180);
	//fill mapA : 1 corresponds to edges , 0 corresponds to gates
	//DriveController uDrive;
	//uDrive.DriveDistance(30);
	//Point currLoc(30,30);
	//int currAngle = 270;
	//localize(currLoc, currAngle);
	//goToGoal(currLoc, goal_definiteVirus);
	//currLoc = goal_definiteVirus;
	//write code to check color
	/*if (color == RED) {
		goToGoal(currLoc, goal_redLab);	
		currLoc = goal_redLab;
	}
	else {
		goToGoal(currLoc, goal_blueLab);
		currLoc = goal_blueLab;		
	}

	goToGoal(goal_possibleVirus1);
	currLoc = goal_possibleVirus1;
	if (color == NONE) {
		goToGoal(goal_possibleVirus2);
		currLoc = goal_possibleVirus2;
	}
	
	if (color == RED) {
		goToGoal(currLoc, goal_redLab);	
		currLoc = goal_redLab;
	}
	else {
		goToGoal(currLoc, goal_blueLab);
		currLoc = goal_blueLab;		
	}*/

	return 0;
}


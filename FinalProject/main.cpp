#include <iostream>
#include <vector>
#include "Util_Map.h"
#include "Particle.h"
#include "sonar.h"
#include "raycaster.h"
#include <wiringPi.h>
#include <math.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "A_Star.h"
#include "navigation.h"
#include "localize.h"

using namespace cv;
using namespace std;

class Point {
public:
	int x;
	int y;
	Point(int x, int y) {
		this.x = x;
		this.y = y;
	}
};


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

int main(){
	//fill mapA : 1 corresponds to edges , 0 corresponds to gates
	//DriveController uDrive;
	//uDrive.DriveDistance(30);
	Point currLoc(30,30);
	int currAngle = 270;
	//localize(currLoc, currAngle);
	goToGoal(currLoc, goal_definiteVirus);
	currLoc = goal_definiteVirus;
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


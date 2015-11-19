#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "Util_Map.h"
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "Util.h"
#include "Particle.h"
#include "drive.h"
#include "sonar.h"
#include "raycaster.h"
#include <wiringPi.h>
#include <math.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "A_Star.h"


using namespace cv;
using namespace std;

Point goal_definiteVirus;
Point goal_possibleVirus1;
Point goal_possibleVirus2;
Point goal_blueLab;
Point goal_redLab;

const int GRIDSIZE 30
const int BOXSIZE = 301;
int RobotSize = 50;
Point RobotCenter;
int angleRot = 0;
int angle = 90;
int NUM_PARTICLES = 30;
int DIS_VARIANCE = 5;
int ANG_VARIANCE = 0;
int DISTANCE = 5;
std::vector<Particle> vParticles;
std::vector<Point> particlesShoots;
std::vector<Point > edges;
std::vector<Point > doors;
std::vector<Point > docks;
Mat image;

int goToGoal(Point, currLoc, Point goalLoc){
	string path = pathFind(currLoc.x currLoc.y, goalLoc.x ,goalLoc.y);
	for(char i=0; i<path.length(); i++) {
		int dir = path[i] - '0';
		Point nextLoc(currLoc.x + dx[i]*GRIDSIZE, currLoc.y + dy[i]*GRIDSIZE);
		moveTo(currLoc, nextLoc);
		currLoc = nextLoc;
	}
}

int main(){
	//fill mapA : 1 corresponds to edges , 0 corresponds to gates
	Point currLoc = localize();
	goToGoal(currLoc, goal_definiteVirus);
	currLoc = goal_definiteVirus;
	//check color
	if (color == RED) {
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
	}

	return 0;
}


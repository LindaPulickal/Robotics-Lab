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

using namespace cv;
using namespace std;

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

#define drawCross( center, color, d )                  \
line( image, cv::Point( center.x - d, center.y - d ),           \
cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0);   \
line( image, cv::Point( center.x + d, center.y - d ),           \
cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

void moveRobot(cv::Point& pos, float distance){	
	pos.x += distance*cos(((float)angle)*M_PI/180);
	pos.y -= distance*sin(((float)angle)*M_PI/180);
	for(int i=0; i<NUM_PARTICLES; i++){
		vParticles[i].moveParticle(vParticles[i].getAngle(), distance, DIS_VARIANCE);
	}
}

Point getIntersectionWithWall(Point pos, double theta){
	Point intersection = RayCasterIntersection(pos, theta, edges);
	return intersection;
}

void rotateImage(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
}

void rotateRobot(cv::Mat& src, cv::Mat& dst) {
    angleRot = angleRot % 360;
    angle += angleRot;
		if(angle < 0){
			angle = 360 + angle;
		}
    angle = angle % 360; 
	//	rotateImage(src, angleRot, dst);
		for(int i=0; i<NUM_PARTICLES; i++){
			//cout << "particleOldAng: " << vParticles[i].getAngle() << endl;			
			vParticles[i].setAngle(vParticles[i].getAngle() + angleRot);
			if(vParticles[i].getAngle() < 0){ //lower bound
				vParticles[i].setAngle(360 + vParticles[i].getAngle());
			}
			if(vParticles[i].getAngle() > 360){
				vParticles[i].setAngle(vParticles[i].getAngle() - 360);
			}
			vParticles[i].rotateParticle(vParticles[i].getAngle(), ANG_VARIANCE);
			//cout << "particleNewAng: " << vParticles[i].getAngle() << endl;
		}
		//cout << "angleRot: " << angleRot << endl;
		//cout << "angle: " << angle << endl;
}

void updateProbability(std::vector<Particle> &particles, std::vector<cv::Point> &particlesShoot, double distance);
std::vector<Particle> resampleParticles(std::vector<Particle>& oldParticles);

int main (int argc, char * const argv[]) {
    char codeChar;
    bool draw = true;
    Mat image2;
    image = Mat::zeros(BOXSIZE, BOXSIZE, CV_8UC3);
    image = cv::Scalar::all(0);

    Point arcReactorLoc;
    parseFile(edges,doors,docks,arcReactorLoc);
    drawMap(image, edges, doors, docks, arcReactorLoc);
    cv :: Mat logo = cv :: imread ("ironman_icon.jpg");
    cv::Point pos(50, 50);
    cv :: Mat imageROI;
    imageROI = image (cv::Rect (pos.x, pos.y, logo.cols, logo.rows));
		RobotSize = logo.cols;
		RobotCenter = Point(pos.x + RobotSize/2, pos.y + RobotSize/2);
    logo.copyTo (imageROI);
    cv :: namedWindow("result");
    cv :: imshow ("result", image);
    cv :: waitKey (0);
	Point robotShoot;

	//Seed random number generator
	srand(time(NULL));

	//create particleShoots
	for(int i=0; i<NUM_PARTICLES; i++){
		particlesShoots.push_back(Point(0, 0));
	}

	//create particles
	for(int i=0; i<NUM_PARTICLES; i++){
		vParticles.push_back(Particle(Point(rand() % BOXSIZE + 1, rand() % BOXSIZE + 1), 0.0, 1.0));//((rand() % 360)*0+90 + 1), 1.0));
	}

	//initialize sonar
	SonarInit();

	
   while(1){

    codeChar = (char)cv::waitKey(100);      
		RobotCenter.x = pos.x + RobotSize/2;
		RobotCenter.y = pos.y + RobotSize/2; 

    if (codeChar == 'w') { //up arrow
	    moveRobot(pos, DISTANCE);   
	    uDrive.DriveDistance(5);    
		}
		if (codeChar == 's'){ //down arrow
			moveRobot(pos, -DISTANCE);
			uDrive.DriveDistance(-5);
		}
	//	else{
	 //   uDrive.DriveDistance(5);
	//	}
        
    if (codeChar == 'i') {
	   // robotShoot = getIntersectionWithWall(RobotCenter, angle);
	    //int distance = distToEdge(RobotCenter, robotShoot);
			//cout << "distance: " << distance << endl;
	    int distance = SonarGetCM();
			for(int i=0; i<NUM_PARTICLES; i++){
      	particlesShoots[i] = getIntersectionWithWall(vParticles[i].getPosition(), vParticles[i].getAngle());
	    }
	    updateProbability(vParticles, particlesShoots, distance); 
		//	vParticles = resampleParticles(vParticles);
    }
		if (codeChar == 'r'){
			vParticles = resampleParticles(vParticles);
		}
    if (codeChar == 'a') { //left arrow
      angleRot = -90;
      rotateRobot(logo, logo);
	//DriveRotational(10);
	  	uDrive.DriveTurn(-90);
    }    
    if (codeChar == 'd') { //right arrow
      angleRot = 90;
      rotateRobot(logo, logo);
     //DriveRotational(-10);
	  	uDrive.DriveTurn(90);
    }
    if (codeChar  == 'j') {
	angleRot = 45;
	rotateRobot(logo, logo);
	uDrive.turnSonar(angle);
	}    
    if (codeChar  == 'k') {
	angleRot = -45;
	rotateRobot(logo, logo);
	uDrive.turnSonar(angle);
	}    

    if (draw) {
	    image = Mat::zeros(BOXSIZE, BOXSIZE, CV_8UC3); 
      drawMap(image, edges, doors, docks, arcReactorLoc);
	    image2 = image;
      imageROI = image2 (cv :: Rect (pos.x, pos.y, logo.cols, logo.rows));
      logo.copyTo (imageROI);
      //cv :: namedWindow("result");
			for(int i=0; i<=NUM_PARTICLES; i++){	
				drawCross(vParticles[i].getPosition(), Scalar(100, 0, 0), 2);
				drawCross(particlesShoots[i], Scalar(100, 0, 100), 2);
			}
			drawCross(robotShoot, Scalar(0, 100, 0), 10);
      cv :: imshow ("result", image2);
      } 
    }
    return 0;
}

void updateProbability(std::vector<Particle> &particles, std::vector<cv::Point> &particlesShoot, double distance) {
    
    float total_probabilities = 0.0;
    float new_weight = 0.0;
    float old_probabilities = 0.0;
    float new_probability = 0.0;
    double map_distance = 0.0;
    double sonar_variance = 10.0;
    
    // update all the particle probabilities
    for (int i=0; i<particles.size(); i++){
        cv::Point pos = particles[i].getPosition();
        
        // use heading to calculate the map distance from particle to wall.
        //map_distance =  distToEdge(direction, cv::Point(pos));
        map_distance = distToEdge(particles[i].getPosition(), particlesShoot[i]);
        
        // Compute new probability using measured distance , map distance and sonar variance
        new_probability =  getProbability(distance, sonar_variance, map_distance); //distance by sonar report, sonar variance, given loaction
				//cout << "newprob: " << new_probability << endl;
        // update each probability and compute total probabilities
        old_probabilities = particles[i].getWeight(); //P(robot@location)
        new_weight = old_probabilities * new_probability; //P(Sensor Reading| Robot @ Location) * P(robot@location)
        particles[i].setWeight(new_weight);
	//			cout << "new_weight: " << new_weight << endl;
        total_probabilities += new_weight; //Ex: 0.25 + 0.25 + 0.3 = 0.8, so N = 1/0.8
    }
    
    
    // Normalize all probabilities
    for (int i=0; i<particles.size(); i++){
        //normalized probability = probability / total probabilities
        particles[i].setWeight(particles[i].getWeight()/total_probabilities); //0.25/0.8 + 0.25/0.8 + 0.3/0.8 = 1
			//cout << "prob: " << particles[i].getWeight() << endl;    
		}
    
}


std::vector<Particle> resampleParticles(std::vector<Particle>& oldParticles) {
    std::vector<Particle> newParticles;
    
    //Calculate a Cumulative Distribution Function for our particle weights
    std::vector<double> CDF;
    CDF.resize(oldParticles.size());
    CDF[0] = ((Particle)oldParticles[0]).getWeight();
    
    for(int i=1; i<CDF.size(); i++)
        CDF[i] = CDF[i-1] + oldParticles[i].getWeight();
    //Loop through our particles as if spinning a roulette wheel.
    //The random u that we start with determines the initial offset
    //and each particle will have a probability of getting landed on and
    //saved proportional to its posterior probability. If a particle has a very large
    //posterior, then it may get landed on many times. If a particle has a very low
    //posterior, then it may not get landed on at all. By incrementing by
    // 1/(numParticles) we ensure that we don't change the number of particles in our
    // returned set.
    
    int i = 0;
    double u = randomDouble()* 1.0/double(oldParticles.size());
    
    for(int j=0; j < oldParticles.size(); j++){
        while(u > CDF[i]) //if i particle is very small, we don't want to let it in newparticle, so i++
            i++;
        //cout << "i = " << i << endl;
        
        Particle p = oldParticles[i]; //kill ridiculos particles, and leave possible particles
        p.setWeight(1.0/double(oldParticles.size()));
        newParticles.push_back(p);
        //cout << " particles[" << i << "].x_2 = " << newParticles[i].getPosition().x << endl;
        
        u += 1.0/double(oldParticles.size());
        //cout << "u = " << u << endl;
    }

    
    return newParticles;
}


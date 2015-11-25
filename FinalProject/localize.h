#include "Util.h"
#define NUM_PARTICLES 30
#define BOXSIZE 301

#define drawCross( center, color, d )                  \
line( image, cv::Point( center.x - d, center.y - d ),           \
cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0);   \
line( image, cv::Point( center.x + d, center.y - d ),           \
cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

std::vector<Particle> vParticles;
std::vector<Point> particlesShoots;
std::vector<Point> edges, docks;

bool getPosition(Point &location, int & angle) {
	int Rx = 0, Ry = 0, Rt = 0, Vx = 0, Vy = 0, Vt = 0;
	
	for(int i=0; i<NUM_PARTICLES; i++){
		Point pos = vParticles[i].getPosition();
		int weight = vParticles[i].getWeight();
		int theta = vParticles[i].getAngle();
		Rx += pos.x * weight;
		Ry += pos.y * weight;
		Rt += theta * weight;
	}
	for(int i=0; i<NUM_PARTICLES; i++){
		Point pos = vParticles[i].getPosition();
		int weight = vParticles[i].getWeight();
		int theta = vParticles[i].getAngle();
		Vx += (pos.x - Rx) * (pos.x - Rx) * weight;
		Vy += (pos.y - Ry) * (pos.y - Ry) * weight;
		Vt += (theta - Rt) * (theta - Rt) * weight;
	}
	if (Vx <= 10 && Vy <= 10 && Vt <=10) {
		location.x = Rx;
		location.y = Ry;
		angle = Rt;
		return true;
	}
	return false;
}



Point getIntersectionWithWall(Point pos, double theta) {
	Point intersection = RayCasterIntersection(pos, theta, edges);
	return intersection;
}

void updateProbability(std::vector<Particle> &particles, std::vector<cv::Point> &particlesShoot, double distance);
std::vector<Particle> resampleParticles(std::vector<Particle>& oldParticles);

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

void localize(Point& currPoint, int& angle) {
    srand(time(NULL));

	//create particleShoots
	for(int i=0; i<NUM_PARTICLES; i++){
		particlesShoots.push_back(Point(0, 0));
	}

	//create particles
	for(int i=0; i<NUM_PARTICLES; i++){
		vParticles.push_back(Particle(Point(rand() % BOXSIZE + 1, rand() % BOXSIZE + 1), 0.0, 1.0));//((rand() % 360)*0+90 + 1), 1.0));
	}

	SonarInit(); //initialize sonar

   	while(1){
		int distance = SonarGetCM();
		for(int i=0; i<NUM_PARTICLES; i++){
      		particlesShoots[i] = getIntersectionWithWall(vParticles[i].getPosition(), vParticles[i].getAngle());
	    }
	    updateProbability(vParticles, particlesShoots, distance); 
		vParticles = resampleParticles(vParticles);
		if (getPosition(currPoint, angle))
			break;
	}		
    
}


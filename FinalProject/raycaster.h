#include <iostream>
#include <math.h>

using namespace std;

#define MAX_DISTANCE 100000000

//INPUTS: 
//ray (starting point, direction)
//array of edges
//OUTPUT:
//nearest intersection point

double RayCasterCross(Point A, Point B){
return A.x*B.y - A.y*B.x;
}

double RayCasterDot(Point A, Point B){
return A.x*B.x + A.y*B.y;
}

Point RayCasterIntersection(Point ray, double angle, std::vector<Point > edges){
double minDistance = MAX_DISTANCE;
Point closestInter;
double rad = (double)angle*M_PI/180.0;

	for(int i=0; i<edges.size()-1; i+=2){	
		double distance;
		Point intersection;
		Point edgeA = edges[i];
		Point edgeB = edges[i+1];
		Point v1 = ray - edgeA;
		Point v2 = edgeB - edgeA;
		Point v3 = Point2f(sin(rad), cos(rad));

		double t1 = RayCasterCross(v2, v1)/RayCasterDot(v2, v3);
		double t2 = RayCasterDot(v1, v3)/RayCasterDot(v2, v3);
		Point direction = Point2f(cos(rad), -sin(rad));
		intersection = ray + direction * t1;		
		distance = norm(ray - intersection);
		if(t2 >= 0 && t2 <= 1 && t1 >= 0 && distance < minDistance){ //t1, t2 in valid ranges and the closest point
			minDistance = distance;
			closestInter = intersection;
		}		
	}
	return closestInter;
}

#include<math.h>
#include "drive.h"

int moveTo(Point currPoint, int currAngle, Point wayPoint){
	int angleRotate = atan2((wayPoint.y - currPoint.y)/(wayPoint.x - currPoint.x)) - currAngle;
	DriveTurn(angleRotate);
	DriveDistance(sqrt(pow(wayPoint.x - currPoint.x, 2) + pow(wayPoint.y - currPoint.y, 2));
}


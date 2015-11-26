#include<math.h>
#include "drive.h"

int moveTo(Point currPoint, float &currAngle, Point wayPoint, int direction){
	//float angleRotate = atan2((double)(-wayPoint.y + currPoint.y),(double)(wayPoint.x - currPoint.x)) - currAngle;
	//double dot = currPoint.x * wayPoint.x + currPoint.y * wayPoint.y;
	//double det = currPoint.x * wayPoint.y - currPoint.y * wayPoint.x; 
	//float angleRotate = atan2(det, dot) - currAngle;
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
	driver.DriveDistance()
	//driver.DriveDistance((float)(sqrt(pow((double)(wayPoint.x - currPoint.x), 2) + pow((double)(wayPoint.y - currPoint.y), 2))));
}


void moveRobot(cv::Point& pos, float distance) {	
	pos.x += distance*cos(((float)angle)*M_PI/180);
	pos.y -= distance*sin(((float)angle)*M_PI/180);
	for(int i=0; i<NUM_PARTICLES; i++){
		vParticles[i].moveParticle(vParticles[i].getAngle(), distance, DIS_VARIANCE);
	}
}


void rotateImage(cv::Mat& src, double angle, cv::Mat& dst) {
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
	for(int i=0; i<NUM_PARTICLES; i++){
		vParticles[i].setAngle(vParticles[i].getAngle() + angleRot);
		if(vParticles[i].getAngle() < 0){ //lower bound
			vParticles[i].setAngle(360 + vParticles[i].getAngle());
		}
		if(vParticles[i].getAngle() > 360){
			vParticles[i].setAngle(vParticles[i].getAngle() - 360);
		}
		vParticles[i].rotateParticle(vParticles[i].getAngle(), ANG_VARIANCE);
	}
}

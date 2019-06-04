#include "MyWindow.hpp"
#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::math;

bool controlBit = false;
bool isOpen = true;
int mIKCountDown = 0;
int mPreCountDown = 0;
int steps;
int presteps;
int flag1 = 1;
int flag2 = 1;
int flag3 = 1;
int flag4 = 0;

std::vector<Eigen::Vector3d> point;
Eigen::Vector3d pretarget;
Eigen::VectorXd targetpose;
Eigen::VectorXd currentpose;
std::vector<Eigen::VectorXd> target_plus;
Eigen::Vector3d prev_ball;
Eigen::Vector3d original_ball;
Eigen::Vector3d max_ball;


MyWindow::MyWindow(const WorldPtr& world) : SimWindow(), mForceCountDown(0), mPoseCountDown(-1)
{	
	this->setWorld(world);
	mWorld->setGravity(Eigen::Vector3d(0,-9.81,0.0));
	initSkeleton();
	std::vector<Tendon*> tempTendon;
	for(int i = 0 ; i < mFingerTendon.size();++i){
		tempTendon.push_back(mFingerTendon[i].second);
	}
	defaultPose = hand->getPositions();
	mController = dart::common::make_unique<Controller>(
		mWorld->getSkeleton("hand"),tempTendon);

	dyn = new Dynamics(mWorld, Eigen::Vector3d(0.0,4.0, 0.0));
	dyn->optimize("Three_Ctrl_4_stepping_Trial_Pose");

	//Pregrabbing algorithm
	currentpose = mController->mTargetPositions;
	targetpose = mController->grabOrOpen(currentpose, isOpen);
	isOpen = false;

	std::vector<Eigen::VectorXd> seriesPose;
	double T = 0.2;
	steps = T / mWorld->getTimeStep();
	presteps = 0.2 / mWorld->getTimeStep();
	int timingOpen = steps *  90/ 100;
	for(int i = 0; i < timingOpen ; ++i)
		seriesPose.push_back(targetpose);
	for(int i = timingOpen; i < steps; ++i){
		Eigen::VectorXd interpolate_pose = currentpose;
		if (i < timingOpen + 5){
			for(int j = 0; j < currentpose.size(); ++j)
				interpolate_pose[j] = currentpose[j] + (targetpose[j] - currentpose[j]) * (timingOpen+4-i) / 4;
		}
		else
			interpolate_pose = currentpose;

		seriesPose.push_back(interpolate_pose);
	}

	//From here we manually gives the output velocity.
	Eigen::VectorXd controlPts = Eigen::VectorXd::Zero(3);
	controlPts[0] = -2.79625;
	controlPts[1] = -1.77311;
	controlPts[2] = -0.775257;

	Eigen::VectorXd controlPts2 = Eigen::VectorXd::Zero(3);
	controlPts2[0] = 0.375923;
	controlPts2[1] = 0.500509;
	controlPts2[2] = 0.0;

	Eigen::VectorXd controlPts3 = Eigen::VectorXd::Zero(3);
	controlPts3[0] = -0.298132;
	controlPts3[1] = -0.749066;
	controlPts3[2] = 0.4;

	// controlPts[3] = -1.843;
	// controlPts[4] = -1.71848;
	// controlPts[5] = -1.54651;

	for(int i = 0; i< steps; ++i){
		Eigen::VectorXd tempPose = seriesPose[i];
		// double current_t = mWorld->getTimeStep() * i;
		double current_t = mWorld->getTimeStep() * i / T;
		tempPose[2] = 0;
		tempPose[4] = 0;
		tempPose[5] = 0;
		int m = controlPts.size();
		for(int j =0; j < m; ++j){
			tempPose[2] += controlPts[j]*dyn->combination(m-1,j) * pow((1-current_t), (m-1-j)) * pow(current_t, j);
			tempPose[4] += controlPts2[j]*dyn->combination(m-1,j) * pow((1-current_t), (m-1-j)) * pow(current_t, j);
			tempPose[5] += controlPts3[j]*dyn->combination(m-1,j) * pow((1-current_t), (m-1-j)) * pow(current_t, j);  
		}
		target_plus.push_back(tempPose);		
	}
	 // This is the last line to be commentted

	setPretarget();
	original_ball = ball->getCOM();

	target_plus = dyn->poseGetter();


}


void MyWindow::setPretarget(){
	Eigen::VectorXd pose = hand->getPositions();
	for(int i = 0 ; i< 4 ; ++i){
		pose[i*4 + 7] = radian(66.0);
		pose[i*4 + 8] = radian(43.0);
		pose[i*4 + 9] = radian(58.0);
	}
	pose[22] = radian(90.0);
	pose[23] = radian(-40.0);
	pose[24] = radian(30.0);
	pose[25] = radian(40.0);
	pose[26] = radian(30.0);
	hand->setPositions(pose);
}

void MyWindow::setTarget(){


}

void MyWindow::initSkeleton(){
	floor  = Skeleton::create("floor");
	hand = Skeleton::create("hand");
	ball = Skeleton::create("ball");

	SkelParser skelP;
	skelP.makeFloor(floor, "floor");
	// skelP.makeBall(ball);
	skelP.makeCylinder(ball, 0.03, 0.03);

	HandMaker handMaker;
	handMaker.makeHand(hand);

	mFingerTendon = handMaker.fingerTendon;

	// mWorld->addSkeleton(floor);
	mWorld->addSkeleton(hand);
	mWorld->addSkeleton(ball);
	poseSetter();
}

void MyWindow::poseSetter()
{
	double off = 0.008;
	Eigen::VectorXd pose = hand->getPositions();
	pose[2] = radian(-90);
	hand->setPositions(pose);
	ball->setPosition(3, hand->getBodyNode("palm")->getCOM()[0] + off);
	// ball->setPosition(3, 10.0);
	// ball->setPosition(4, 0.9);
	//ball->setPosition(5, 0.001);
}


void MyWindow::initSkeletonFinger()
{
	finger = Skeleton::create("finger");
	floor  = Skeleton::create("floor");

	SkelParser skelP;
	skelP.makeFloor(floor, "floor");
	skelP.makeFinger(finger);
	//skelP.sphereJointVis(skel, visual_sphere);

	double tendon_angle = 30.0;

	skelP.setGeometry(finger, "L2", -tendon_angle * 1.0, 0);
	skelP.setGeometry(finger, "L2", tendon_angle, 2);
	skelP.setGeometry(finger, "L3", tendon_angle, 0);
	skelP.setGeometry((finger), "L4", tendon_angle, 0);

	for(std::size_t i = 0; i < finger->getNumJoints(); ++i)
		finger->getJoint(i)->setPositionLimitEnforced(true);

	mWorld->addSkeleton(finger);
	mWorld->addSkeleton(floor);
}

double MyWindow::radian(double angle){
	double rad = angle * M_PI / 180;
	return rad;
}

void MyWindow::targetMovement()
{
	//std::cout << "enter" << std::endl;
	goalPose = defaultPose;
	goalPose[1] = radian(30);
	goalPose[3] = -3.0;
	//std::cout << goalPose << std::endl;
}

void MyWindow::basicMovement(){
	double time = mWorld->getTime();
	if(controlBit){
		Eigen::VectorXd pose = defaultPose;
		double basic_angle = sin(time)* 20 * M_PI / 180;
		pose[0] = basic_angle;
		basic_angle = 20 * M_PI / 180;
		pose[2] = basic_angle;
		basic_angle = 40 * M_PI / 180;
		pose[3] = basic_angle;
		basic_angle = 15 * M_PI / 180;
		pose[4] = basic_angle;
		mController->setTargetPosition(pose);
	}
	else{
		mController->setTargetPosition(defaultPose);
	}
}

void MyWindow::initTendonFinger()
{
	std::string name = "z-axis";
	std::size_t i;
	for(i = 0 ; i < finger->getNumJoints()-1 ; ++i){
		mTendon.push_back(new Tendon(name + std::to_string(i)));
	}
	Eigen::Vector3d down_link(default_width/2, -default_link_len/6, 0);
	Eigen::Vector3d down_dir(0, -default_link_len/6, 0);
	Eigen::Vector3d upper_link(default_width/2, default_link_len/6, 0);
	Eigen::Vector3d upper_dir(0, default_link_len/6, 0);
	Eigen::Vector3d null_dir(0,0,0);

	down_dir.normalize();
	upper_dir.normalize();

	for(i = 0 ; i < finger->getNumJoints()-1 ; ++i){
		BodyNode* bn = finger->getBodyNode(i);
		mTendon[i]-> AddAnchor(bn, upper_link, upper_dir, null_dir, false);
		bn = finger->getBodyNode(i+1);
		mTendon[i]-> AddAnchor(bn, down_link, null_dir, down_dir, false);
	}

	name = "x-axis";
	for(std::size_t j = 0 ; j < 2; ++j){
		mTendon.push_back(new Tendon(name + std::to_string(j)));
	}
	BodyNode* bn;
	down_link = Eigen::Vector3d(0, -default_link_len/6, default_width/2);
	upper_link = Eigen::Vector3d(0, default_link_len/6, default_width/2);


	bn = finger->getBodyNode(0);
	mTendon[i] -> AddAnchor(bn, upper_link, upper_dir, null_dir, false);
	bn = finger->getBodyNode(1);
	mTendon[i]-> AddAnchor(bn, down_link, null_dir, down_dir, false);

	down_link = Eigen::Vector3d(0, -default_link_len/6, -default_width/2);
	down_dir = Eigen::Vector3d(0, -default_link_len/6, 0);
	upper_link = Eigen::Vector3d(0, default_link_len/6, -default_width/2);
	upper_dir = Eigen::Vector3d(0, default_link_len/6, 0);
	down_dir.normalize();
	upper_dir.normalize();

	bn = finger->getBodyNode(0);
	mTendon[i+1] -> AddAnchor(bn, upper_link, upper_dir, null_dir, false);
	bn = finger->getBodyNode(1);
	mTendon[i+1]-> AddAnchor(bn, down_link, null_dir, down_dir, false);
	
}

void MyWindow::showTorque()
{
	std::cout << "pose" << std::endl;
	std::cout << hand->getPositions() << std::endl;
}

  /// Handle keyboard input
void MyWindow::keyboard(unsigned char key, int x, int y)
{
	Eigen::VectorXd pose;
	switch(key)
	{
		case 'q':
		isOpen = !isOpen;
		std::cout << isOpen << std::endl;
		currentpose = mController->mTargetPositions;
		mPoseCountDown = default_countdown_movement;
		targetpose = mController->grabOrOpen(currentpose, isOpen);
		// mController->setTargetPosition(mController->grabOrOpen(ball, defaultPose, isOpen));
		// isOpen = !isOpen;
		break;
		case 'a':
		pose = mController->mTargetPositions;
		pose[2] += radian(5);
		mController->setTargetPosition(pose);
		break;
		case 's':
		pose = mController->mTargetPositions;
		pose[2] -= radian(5);
		mController->setTargetPosition(pose);
		break;
		case 'd':
		pose = mController->mTargetPositions;
		pose[4] += radian(5);
		mController->setTargetPosition(pose);
		break;
		case 'f':
		pose = mController->mTargetPositions;
		pose[4] -= radian(5);
		mController->setTargetPosition(pose);
		break;
		case 'x':
		mPreCountDown = 5000;
		// MyWindow::basicMovement();
		// controlBit = !controlBit;
		break;

		default:
		SimWindow::keyboard(key, x, y);
	}
}

void MyWindow::timeStepping() 
{
	mController->clearForces();
	mController->addSPDForces();
	// mController->addSPDTendonDirectionForces();

	// if(flag1 < presteps + 1){
	// 	Eigen::VectorXd pose = currentpose;	
	// 	for(int j = 0 ; j < targetpose.size();++j){
	// 		pose[j] = currentpose[j] + (targetpose[j] - currentpose[j]) * flag1 / presteps;
	// 	}
	// 	mController->setTargetPosition(pose);
	// 	flag1++;
	// }
	// else if(flag1 < presteps * 2) flag1++;


	if(flag2 < presteps + 1){
		Eigen::VectorXd new_pose = target_plus[0];
		Eigen::VectorXd pose = targetpose;	
		for(int j = 0 ; j < targetpose.size();++j){
			pose[j] = targetpose[j] + (new_pose[j] - targetpose[j]) * flag2 / presteps;
		}
		mController->setTargetPosition(pose);
		// std::cout << "step 2" << std::endl;
		// std::cout << ball->getCOMLinearVelocity() << std::endl;
		flag2++;
	}
	// else if(flag3 < presteps + 1){
	// 	flag3++;
	// 	if(flag3 == presteps){
	// 		std::cout << "ball position" << std::endl;
	// 		std::cout << ball->getPositions() <<std::endl;
	// 		std::cout << "hand position" << std::endl;
	// 		std::cout << hand->getBodyNode("palm")->getCOM() <<std::endl;
	// 	}
	// 	// std::cout << "step 3" << std::endl;
	// 	// std::cout << ball->getCOMLinearVelocity() << std::endl;
	// }
	else if(flag4 < steps){
		mController-> setTargetPosition(target_plus[flag4]);
		// std::cout << "step 4" << std::endl;
		// std::cout << ball->getCOMLinearVelocity() << std::endl;
		flag4++;
	}
	else if (flag4 == steps) {
		// std::cout << "velocity is " << std::endl;
		// std::cout << ball->getCOMLinearVelocity() << std::endl;
		prev_ball = ball->getCOM();
		flag4++;
	}
	else if(flag4 < steps * 10) 
	{
		Eigen::Vector3d current_ball = ball->getCOM();
		double epsilon = 0.03;
		if(current_ball[1] < prev_ball[1] && std::abs(ball->getCOMLinearVelocity()[1]) < epsilon)
		{
			max_ball = prev_ball;
			flag4 = steps * 10-1;
			std::cout << max_ball <<std::endl;
		}
		else
			prev_ball = current_ball;
		flag4++;
	}
	else if(flag4 == steps * 10)
	{
		if(max_ball[1] < 0) max_ball[1] = 0.000001;
		double t_max = std::sqrt(2 * max_ball[1] * std::abs(mWorld->getGravity()[1])) / std::abs(mWorld->getGravity()[1]);
		Eigen::Vector3d v_ball;
		v_ball[1] = std::sqrt(2 * max_ball[1] * std::abs(mWorld->getGravity()[1]));
		v_ball[0] = max_ball[0] / t_max;
		v_ball[2] = max_ball[2] / t_max;
		std::cout << "v ball is" << std::endl;
		std::cout << v_ball << std::endl;
		flag4++;
		// mWorld->reset();
	}



	if(mPoseCountDown >=0)
	{
		Eigen::VectorXd pose = currentpose;		
		if(mPoseCountDown%50 == 0){			
			for(int i =6; i<targetpose.size();++i)
				pose[i] = currentpose[i] + (targetpose[i] - currentpose[i]) * (default_countdown_movement - mPoseCountDown) / default_countdown_movement;
			mController->setTargetPosition(pose);
		}
		--mPoseCountDown;
	}

	if(mForceCountDown > 0)
	{
		BodyNode* bnn = mWorld->getSkeleton("finger")->getBodyNode("L2");
		bnn->addExtForce(-default_force * Eigen::Vector3d::UnitZ(),
			bnn->getCOM(), false, false);

		--mForceCountDown;
	}
	// basicMovement();

    // Step the simulation forward
	SimWindow::timeStepping();

}

void MyWindow::drawTendon(){
	glPointSize(8.0);
	glLineWidth(8.0); 
	for(int i = 0; i < mTendon.size(); ++i){
		for(int j = 0; j < mTendon[i]->mAnchor_dir.size()-1; ++j){
			Eigen::Vector3d first = mTendon[i]->GetPoint(mTendon[i]->mAnchor_dir[j]);
			Eigen::Vector3d second = mTendon[i]->GetPoint(mTendon[i]->mAnchor_dir[j+1]);
			Eigen::Vector3d first_dir = mTendon[i]->GetDir(mTendon[i]->mAnchor_dir[j], 0);
			Eigen::Vector3d second_dir = mTendon[i]->GetDir(mTendon[i]->mAnchor_dir[j+1], 1);
			Eigen::Vector3d midpoint = getMidPoint(first,second,first_dir, second_dir);

			glColor3f(mTendon[i]->mForce* 0.7 , 0.0, -mTendon[i]->mForce*0.7);

			glBegin(GL_LINE_STRIP);
			glVertex3f(first[0], first[1], first[2]);
			glVertex3d(midpoint[0], midpoint[1], midpoint[2]);
			glVertex3f(second[0], second[1], second[2]);
			glEnd();
		}
	}
}

void MyWindow::drawMultipleTendons(){
	glColor3f(1.0, 0.0, 0.0); 
	glPointSize(8.0);
	glLineWidth(8.0); 
	for(int i = 0; i < mFingerTendon.size(); ++i){
		//std::cout << i <<std::endl;
		for(int j = 0; j < mFingerTendon[i].second->mAnchor_dir.size()-1; ++j){
			//std::cout << i << std::endl;
			Eigen::Vector3d first = mFingerTendon[i].second->GetPoint(mFingerTendon[i].second->mAnchor_dir[j]);
			Eigen::Vector3d second = mFingerTendon[i].second->GetPoint(mFingerTendon[i].second->mAnchor_dir[j+1]);
			Eigen::Vector3d first_dir = mFingerTendon[i].second->GetDir(mFingerTendon[i].second->mAnchor_dir[j], 0);
			Eigen::Vector3d second_dir = mFingerTendon[i].second->GetDir(mFingerTendon[i].second->mAnchor_dir[j+1], 1);
			Eigen::Vector3d midpoint = getMidPoint(first,second,first_dir, second_dir);

			//glColor3f(mFingerTendon[i].second->mForce* 0.2 , 0.0, 0.0);

			glBegin(GL_LINE_STRIP);
			glVertex3f(first[0], first[1], first[2]);
			glVertex3d(midpoint[0], midpoint[1], midpoint[2]);
			glVertex3f(second[0], second[1], second[2]);
			glEnd();
		}
	}
}


Eigen::Vector3d MyWindow::getMidPoint(Eigen::Vector3d first, Eigen::Vector3d second, Eigen::Vector3d first_dir, Eigen::Vector3d second_dir)
{
	Eigen::Vector3d midpoint;
	double t;
	if(first_dir == -second_dir){
		midpoint = (first + second) / 2;
	}
	else if(first_dir[0]*second_dir[1] - first_dir[1] * second_dir[0] != 0){
		t = ((second[0]-first[0])*first_dir[1] - (second[1]-first[1]) * first_dir[0] )/ (first_dir[0]*second_dir[1] - first_dir[1] * second_dir[0]);
		midpoint = second + t * second_dir;
	}
	else if(first_dir[0]*second_dir[2] - first_dir[2] * second_dir[0] != 0){
		t = ((second[0]-first[0])*first_dir[2] - (second[2]-first[2]) * first_dir[0] )/ (first_dir[0]*second_dir[2] - first_dir[2] * second_dir[0]);
		midpoint = second + t * second_dir;
	}
	else if(first_dir[2]*second_dir[1] - first_dir[1] * second_dir[2] != 0){
		t = ((second[2]-first[2])*first_dir[1] - (second[1]-first[1]) * first_dir[2] )/ (first_dir[2]*second_dir[1] - first_dir[1] * second_dir[2]);
		midpoint = second + t * second_dir;
	}
	else{
		midpoint = second;
	}

	return midpoint;
}

void MyWindow::drawTarget(){
	glColor3f(1.0, 0.0, 0.0); 
	glLineWidth(8.0); 
	glBegin(GL_LINE_STRIP);
	for(int i = 0; i < Ends.size(); ++i)
		glVertex3f(point[i][0], point[i][1], point[i][2]);
	glEnd();
}

void MyWindow::draw_TargetPoint(){
	glColor3f(1.0, 0.0, 0.0);
	glPointSize(7.0);
	Eigen::Vector3d point = dyn->targetGetter();
	glBegin(GL_POINTS);
	glVertex3f(point[0], point[1], point[2]);
	glEnd();
}

void MyWindow::draw() 
{
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (!mSimulating) {
		if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
			std::size_t nSkels = mWorld->getNumSkeletons();
			for (std::size_t i = 0; i < nSkels; i++) {
	        // std::size_t start = mWorld->getIndex(i);
	        // std::size_t size = mWorld->getSkeleton(i)->getNumDofs();
				mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
			}	      
			if (mShowMarkers) {
	        // std::size_t sumDofs = mWorld->getIndex(nSkels);
				int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
				for (int i = 0; i < nContact; i++) {
					Eigen::Vector3d v = mWorld->getRecording()->getContactPoint(mPlayFrame, i);
					Eigen::Vector3d f = mWorld->getRecording()->getContactForce(mPlayFrame, i);

					glBegin(GL_LINES);
					glVertex3f(v[0], v[1], v[2]);
					glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
					glEnd();
					mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
					mRI->pushMatrix();
					glTranslated(v[0], v[1], v[2]);
					mRI->drawSphere(0.01);
					mRI->popMatrix();
				}
			}
		}
	} 
	else {glPointSize(10.0);  
		if (mShowMarkers) {
			const auto result =
			mWorld->getConstraintSolver()->getLastCollisionResult();
			for (const auto& contact : result.getContacts()) {
				Eigen::Vector3d v = contact.point;
				Eigen::Vector3d f = contact.force / 10.0;
				glBegin(GL_LINES);
				glVertex3f(v[0], v[1], v[2]);
				glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
				glEnd();
				mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
				mRI->pushMatrix();
				glTranslated(v[0], v[1], v[2]);
				mRI->drawSphere(0.01);
				mRI->popMatrix();
			}
		}
	}


	glEnable(GL_LIGHTING);
	drawWorld();
	glDisable(GL_LIGHTING);

	draw_TargetPoint();

	glEnable(GL_LIGHTING);

	  // display the frame count in 2D text
	char buff[64];
	if (!mSimulating)
	#ifdef _WIN32
		_snprintf(buff, sizeof(buff), "%d", mPlayFrame);
	#else
	std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
	#endif
	else
	#ifdef _WIN32
		_snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
	#else
	std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
	#endif
	std::string frame(buff);
	glColor3f(0.0, 0.0, 0.0);
	drawStringOnScreen(0.02f, 0.02f, frame);
	glEnable(GL_LIGHTING);
}

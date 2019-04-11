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
int mIKCountDown = 0;
int mPreCountDown = 0;

std::vector<Eigen::Vector3d> point;
Eigen::Vector3d pretarget;


MyWindow::MyWindow(const WorldPtr& world) : SimWindow(), mForceCountDown(0)
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
		mWorld->getSkeleton("hand"),tempTendon, JOINT_F);

	targetMovement();
	setPretarget();
	setTarget();

	// ik.IKSingleConfig(temporal, hand, 1);

	// initSkeletonFinger();    
	// initTendonFinger();
	// controlBit = true;
	// defaultPose = finger->getPositions();
	// mController = dart::common::make_unique<Controller>(
	// 	mWorld->getSkeleton("finger"),mTendon, JOINT_F);	
}


void MyWindow::setPretarget(){
	Eigen::Vector3d cylPose = ball->getCOM();
	pretarget[0] = cylPose[0]+ 0.3;
	pretarget[1] = cylPose[1]+ 5.5;
	pretarget[2] = cylPose[2];
}

void MyWindow::setTarget(){
	// for(int i = 0; i < 4 ; ++i){
	// 	point.push_back(Eigen::Vector3d(hand->getBodyNode("revol_up" + std::to_string(i))->getCOM()[0]-2.0-i*0.3, hand->getBodyNode("revol_up" + std::to_string(i))->getCOM()[1]-1.0-0.2*i, hand->getBodyNode("revol_up" + std::to_string(i))->getCOM()[2]));
	// 	Ends.push_back(std::make_pair(point[i] ,i));
	// 	//std::cout<<point[i] <<std::endl;
	// }
	// point.push_back(Eigen::Vector3d(hand->getBodyNode("thumb_revol_up")->getCOM()[0]-1.0,hand->getBodyNode("thumb_revol_up")->getCOM()[1]-0.5, hand->getBodyNode("thumb_revol_up")->getCOM()[2]));
	// Ends.push_back(std::make_pair(point[4],4));

	Eigen::Vector3d cylPose = ball->getCOM();
	double pointy = cylPose[1] + 1.2;
	double theta = radian(60);
	double offset = radian(-10);
	int i;
	for(i = 0; i < 5; ++i){
		point.push_back(Eigen::Vector3d(cylPose[0]-1.35*sin(theta*i+offset), pointy, cylPose[2] - 1.35*cos(theta*i+offset)));
		Ends.push_back(std::make_pair(point[i] ,i));
	}
}

void MyWindow::initSkeleton(){
	floor  = Skeleton::create("floor");
	hand = Skeleton::create("hand");
	ball = Skeleton::create("ball");

	SkelParser skelP;
	skelP.makeFloor(floor, "floor");
	skelP.makeBall(ball);
	// skelP.makeCylinder(ball);

	HandMaker handMaker;
	handMaker.makeHand(hand);

	mFingerTendon = handMaker.fingerTendon;

	//mWorld->addSkeleton(floor);
	mWorld->addSkeleton(hand);
	mWorld->addSkeleton(ball);
	poseSetter();
}

void MyWindow::poseSetter(){
	Eigen::VectorXd pose = hand->getPositions();
	pose[2] = radian(-90);
	hand->setPositions(pose);
	ball->setPosition(3, hand->getBodyNode("palm")->getCOM()[0] + 1.0);
}


void MyWindow::initSkeletonFinger()
{
	finger = Skeleton::create("finger");
	floor  = Skeleton::create("floor");

	SkelParser skelP;
	skelP.makeFloor(floor, "floor");
	skelP.makeFinger(finger);
	//skelP.sphereJointVis(skel, visual_sphere);

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

void MyWindow::initTendonFinger(){
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
	// std::cout <<"Force" << std::endl;
	// std::cout << finger->getExternalForces() << std::endl;
	// std::cout << finger->getExternalForces() << std::endl;
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
		mForceCountDown = default_countdown;
		break;
		case 'a':
		mPoseCountDown = default_countdown_movement;
		controlBit = !controlBit;
		break;
		case 'z':
		pose = hand->getPositions();
		pose[2] -= 20 * M_PI / 180;
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
	
	if(mPreCountDown>0){

		IkSolver ik;
		Eigen::VectorXd newPose = hand->getPositions();
		newPose+= ik.IKMiddle(pretarget, hand, "palm");
		mController->setTargetPosition(newPose);

		--mPreCountDown;
		if(mPreCountDown == 0) mIKCountDown = grad_Iter;
	}

	if(mIKCountDown > 0){
		IkSolver ik;
		Eigen::VectorXd newPose = hand->getPositions();		
		newPose += ik.IKMultiple(hand, Ends, mIKCountDown);
		mController->setTargetPosition(newPose);
		// mController->setTargetPosition(ik.TotalIk(Ends, hand));
		--mIKCountDown;
	}

	if(mPoseCountDown > 0)
	{
		if(mPoseCountDown%50 == 0){
			Eigen::VectorXd targetpose = defaultPose;
			if(controlBit){
				for(int i =0; i<targetpose.size();++i)
					targetpose[i] = defaultPose[i] + (goalPose[i] - defaultPose[i]) * (default_countdown_movement - mPoseCountDown) / default_countdown_movement;
			}
			else{
				for(int i =0; i<targetpose.size();++i)
					targetpose[i] = goalPose[i] + (defaultPose[i] - goalPose[i]) * (default_countdown_movement - mPoseCountDown) / default_countdown_movement;
			}
		//std::cout<< targetpose[3] <<std::endl;
			mController->setTargetPosition(targetpose);
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

	// drawTarget();
	// drawMultipleTendons();
	// drawTendon();
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

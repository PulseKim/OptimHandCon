#include "MyWindow.hpp"
#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::math;

bool controlBit;
Eigen::VectorXd defaultPose;

MyWindow::MyWindow(const WorldPtr& world) : SimWindow(), mForceCountDown(0)
{	
	this->setWorld(world);
	mWorld->setGravity(Eigen::Vector3d(0,-9.81,0.0));
	initSkeleton();
	// initSkeletonFinger();    
	// initTendonFinger();
	// controlBit = true;
	// defaultPose = finger->getPositions();
	// mController = dart::common::make_unique<Controller>(
	// 	mWorld->getSkeleton("finger"),mTendon, JOINT_F);
}

void MyWindow::initSkeleton(){
	floor  = Skeleton::create("floor");
	hand = Skeleton::create("hand");

	SkelParser skelP;
	skelP.makeFloor(floor, "floor");

	HandMaker handMaker;
	handMaker.makeHand(hand);

	mWorld->addSkeleton(floor);
	mWorld->addSkeleton(hand);
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


void MyWindow::basicMovement(){
	if(controlBit){
		Eigen::VectorXd pose = defaultPose;
		double basic_angle = 0 * M_PI / 180;
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
	std::cout << finger->getPositions() << std::endl;
}

  /// Handle keyboard input
void MyWindow::keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		case 'q':
		mForceCountDown = default_countdown;
		break;
		case 'a':
		for(std::size_t i =1 ; i < mTendon.size(); ++i)
			mTendon[mTendon.size()-1]->ApplyForceToBody();
		break;
		case 'z':
		MyWindow::showTorque();
		break;
		case 'x':
		MyWindow::basicMovement();
		controlBit = !controlBit;
		break;

		default:
		SimWindow::keyboard(key, x, y);
	}
}

void MyWindow::timeStepping() 
{
	// mController->clearForces();
	// mController->addSPDTendonDirectionForces();

	if(mForceCountDown > 0)
	{
		BodyNode* bnn = mWorld->getSkeleton("finger")->getBodyNode("L2");

		bnn->addExtForce(-default_force * Eigen::Vector3d::UnitZ(),
			bnn->getCOM(), false, false);

		--mForceCountDown;
	}

    // Step the simulation forward
	SimWindow::timeStepping();

}

void MyWindow::drawTendon(){
	glColor3f(1.0, 0.0, 0.0); 
	glPointSize(8.0);
	glLineWidth(8.0); 
	for(int i = 0; i < mTendon.size(); ++i){
		for(int j = 0; j < mTendon[i]->mAnchor_dir.size()-1; ++j){
			Eigen::Vector3d first = mTendon[i]->GetPoint(mTendon[i]->mAnchor_dir[j]);
			Eigen::Vector3d second = mTendon[i]->GetPoint(mTendon[i]->mAnchor_dir[j+1]);
			Eigen::Vector3d first_dir = mTendon[i]->GetDir(mTendon[i]->mAnchor_dir[j], 0);
			Eigen::Vector3d second_dir = mTendon[i]->GetDir(mTendon[i]->mAnchor_dir[j+1], 1);
			Eigen::Vector3d midpoint = getMidPoint(first,second,first_dir, second_dir);


			glBegin(GL_POINTS);
			glVertex3f(first[0], first[1], first[2]);
			glVertex3f(second[0], second[1], second[2]);
			glEnd();

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
	//drawTendon();
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

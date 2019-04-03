#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "Tendon.hpp"

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::math;

const double tendon_angle = 30.0;

enum {JOINT_F=0, LINK_F};

class Controller{
public:
	//Constructor
	Controller(const SkeletonPtr& finger, std::vector<Tendon*> tendon, int type);
	void jointControlSetter();
	void setTargetPosition(const Eigen::VectorXd& pose);
	void clearForces();
	void addSPDForces();
	void addPDForces();
	void addSPDTendonForces();
	void addSPDTendonDirectionForces();
	double prevTorque(const Eigen::Vector3d current_point);




protected:
	//Finger model used to control
	SkeletonPtr mFinger;

	//Tendon model used to control
	std::vector<Tendon*> mTendon;
	
	//Joint force to the fingers
	Eigen::VectorXd mForces;

	Eigen::VectorXd Ycontroller;

	//Tendon force to the fingers
	Eigen::VectorXd mTendonForces;

	//control gains for proportional error terms in the PD controller
	Eigen::MatrixXd mKp;

	//Control gains for the derivative error terms in the PD controller
	Eigen::MatrixXd mKd;

	//Target positions for the PD controllers
	Eigen::VectorXd mTargetPositions;

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> prevForces;
};
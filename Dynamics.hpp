#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "Controller.hpp"
#include <fstream>
#include <string>

using namespace dart::common;
using namespace dart::simulation;


class Dynamics
{
public:
	Dynamics(const WorldPtr& world, const Eigen::Vector3d& v_in);
	double evaluateEnergy(std::vector<Eigen::VectorXd> target_pose);
	double evaluateEnergyPosition(std::vector<Eigen::VectorXd> target_pose);
	std::vector<Eigen::VectorXd> computePose(const Eigen::VectorXd& controlPts, int index);
	int factorial(int i);
	int combination(int n, int i);
	double GDiterate(int index, int iter);
	double GDiteratePose(int index, int iter);
	double SGDiterate(int index, int iter);
	int lambdaLearning(int iter, int lambda);
	void optimize(std::string name);
	void initPose(Eigen::VectorXd initialPose);
	std::vector<Eigen::VectorXd> poseGetter();
	void deleteCollisionAspect();
	void addCollisionAspect();
	double radian(double angle);
	Eigen::Vector3d targetGetter();
	
protected:
	WorldPtr mWorld;
	SkeletonPtr mBall;
	SkeletonPtr mCharacter;
	std::unique_ptr<Controller> mController;
	Eigen::Vector3d v_target;
	std::vector<Eigen::VectorXd> mControlPts;
	std::vector<Eigen::VectorXd> mOldPose;
	Eigen::Vector3d max_pose_target;
};
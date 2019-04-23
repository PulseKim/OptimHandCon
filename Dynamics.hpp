#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "Controller.hpp"

using namespace dart::common;
using namespace dart::simulation;


class Dynamics
{
public:
	Dynamics(const WorldPtr& world, const Eigen::Vector3d& v_in);
	double evaluateEnergy(std::vector<Eigen::VectorXd> target_pose);
	std::vector<Eigen::VectorXd> computePose(const Eigen::VectorXd& controlPts, int index);
	int factorial(int i);
	int combination(int n, int i);
	double iterate(int index, int iter);
	void optimize();

protected:
	WorldPtr mWorld;
	SkeletonPtr mBall;
	SkeletonPtr mCharacter;
	std::unique_ptr<Controller> mController;
	Eigen::Vector3d v_target;
	std::vector<Eigen::VectorXd> mControlPts;
	std::vector<Eigen::VectorXd> mOldPose;
};
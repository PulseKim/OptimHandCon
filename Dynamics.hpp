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
	Dynamics(const WorldPtr& world, double v_in);
	double EvaluateEnergy(const Eigen::VectorXd& param, std::vector<Eigen::VectorXd>oldPose, int index);
	std::vector<Eigen::VectorXd> computePose(const Eigen::VectorXd& param, std::vector<Eigen::VectorXd>oldPose, int index);
	int factorial(int i);
	int combination(int n, int i);

protected:
	WorldPtr mWorld;
	SkeletonPtr mBall;
	std::unique_ptr<Controller> mController;
	double v_target;
};
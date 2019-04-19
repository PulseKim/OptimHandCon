#include "Dynamics.hpp"
#include <math.h>



double T = 1.0;
double step;
int n;

Dynamics::Dynamics(const WorldPtr& world, double v_in): mWorld(world), v_target(v_in)
{
	std::vector<Tendon*> tempTendon;
	mController = dart::common::make_unique<Controller>(
		mWorld->getSkeleton("hand"),tempTendon);
	mBall = mWorld->getSkeleton("ball");
	step = mWorld->getTimeStep();
	n = T/step;
}


int Dynamics::factorial(int i)
{
	if(i == 0) return 1;
	else if(i == 1) return 1;
	return i * factorial(i-1);
}

int Dynamics::combination(int n, int i)
{
	int c = factorial(n) / (factorial(n-i) * factorial(i));
	return c;
}

std::vector<Eigen::VectorXd> Dynamics::computePose(const Eigen::VectorXd& controlPts,std::vector<Eigen::VectorXd>oldPose, int index)
{
	//Evaluate position vectors by Bezier curves
	std::vector<Eigen::VectorXd> target_positions;
	for(int i = 0; i< n ; ++i){
		Eigen::VectorXd tempPose = oldPose[i];
		double current_t = step * i;
		tempPose[index] = 0;
		int m = controlPts.size();
		for(int j =0; j < m; ++j)
			tempPose[index] += combination(m-1,j) * pow((1-current_t), (m-1-j)) * pow(current_t, j); 
		target_positions.push_back(tempPose);
	}
	return target_positions;
}

double Dynamics::EvaluateEnergy(const Eigen::VectorXd& controlPts, std::vector<Eigen::VectorXd>oldPose, int index)
{
	std::vector<Eigen::VectorXd> target_pose = computePose(controlPts, oldPose, index);
	for(int i = 0; i< n ; ++i){
		mController-> setTargetPosition(target_pose[i]);
		mWorld->step();
	}
	double v_ball = mBall->getCOMLinearVelocity()[1];
	return (v_ball-v_target)*(v_ball-v_target);
}
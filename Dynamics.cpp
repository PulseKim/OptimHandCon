#include "Dynamics.hpp"
#include <math.h>



double T = 1.0;
double step;
int n;

Dynamics::Dynamics(const WorldPtr& world, const Eigen::Vector3d& v_in): mWorld(world), v_target(v_in)
{
	std::vector<Tendon*> tempTendon;
	mController = dart::common::make_unique<Controller>(
		mWorld->getSkeleton("hand"),tempTendon);
	mBall = mWorld->getSkeleton("ball");
	mCharacter = mWorld->getSkeleton("hand");	
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

std::vector<Eigen::VectorXd> Dynamics::computePose(const Eigen::VectorXd& controlPts, int index)
{
	//Evaluate position vectors by Bezier curves
	std::vector<Eigen::VectorXd> target_positions;
	for(int i = 0; i< n ; ++i){
		Eigen::VectorXd tempPose = mOldPose[i];
		double current_t = step * i;
		tempPose[index] = 0;
		int m = controlPts.size();
		for(int j =0; j < m; ++j)
			tempPose[index] += combination(m-1,j) * pow((1-current_t), (m-1-j)) * pow(current_t, j); 
		target_positions.push_back(tempPose);
	}
	return target_positions;
}

double Dynamics::evaluateEnergy(std::vector<Eigen::VectorXd> target_pose)
{
	for(int i = 0; i< n ; ++i){
		mController-> setTargetPosition(target_pose[i]);
		mWorld->step();
	}
	Eigen::Vector3d v_ball = mBall->getCOMLinearVelocity();
	double error = 0.0;
	for(int i= 0; i<3; ++i)
		error += (v_target[i] - v_ball[i]) * (v_target[i] - v_ball[i]);
	return error;
}

double Dynamics::iterate(int index, int iter)
{
	//Values saved to be initialized
	Eigen::VectorXd p = mCharacter->getPositions();
	Eigen::VectorXd v = mCharacter->getVelocities();
	Eigen::VectorXd p_ball = mBall->getPositions();
	Eigen::VectorXd v_ball = mBall->getVelocities();

	//Setting Control points
	Eigen::VectorXd controlPts = mControlPts[index];
	Eigen::VectorXd controlPts_plus_eps = controlPts, controlPts_minus_eps = controlPts;
	Eigen::VectorXd grad = Eigen::VectorXd::Zero(controlPts.rows());
	double epslion = 0.01;
	double lambda = 5.0 / std::min(sqrt(iter), 20.0);

	//Calculate gradient
	for(int i = 0; i < controlPts_plus_eps.rows(); ++i){
		controlPts_plus_eps = controlPts;
		controlPts_plus_eps = controlPts;
		controlPts_plus_eps[i] += epslion;
		controlPts_minus_eps[i] -= epslion;

		std::vector<Eigen::VectorXd> target_plus = computePose(controlPts_plus_eps, index);
		std::vector<Eigen::VectorXd> target_minus = computePose(controlPts_minus_eps, index);

		double e_plus = evaluateEnergy(target_plus);
		double e_minus = evaluateEnergy(target_minus);

		grad[i] = (e_plus - e_minus)/0.02 * lambda;
	}

	//Set control points and 
	mControlPts[index] = mControlPts[index] - grad;
	mOldPose = computePose(mControlPts[index], index);

	mWorld->reset();
	mCharacter->setPositions(p);
	mCharacter->setVelocities(v);
	mBall->setPositions(p_ball);
	mBall->setVelocities(v_ball);

	return grad.norm();
}

void Dynamics::optimize(){
	//How to deal with the pre-grabbing?????

	//Initialize mControl, mOld, control indexes
	std::vector<int> control_index;

	int numb_control = 6;
	mControlPts.clear();
	for(int i = 0; i < mCharacter->getPositions().size(); ++i)
		mControlPts.push_back(Eigen::VectorXd::Zero(numb_control));
	for(int i = 0; i < n ; ++i)
		mOldPose.push_back(mCharacter->getPositions());

	//To Do: Implement grab / open, initial controlpoints 


	//Initializing old pose
	for(int i = 0; i < control_index.size(); ++i){
		Eigen::VectorXd currentControl = mControlPts[control_index[i]];
		mOldPose = computePose(currentControl, control_index[i]);
	}

	//Parameters
	double epsilon = 0.001;
	int mIter = 10000;

	//Iterate for each control indexes
	//Stop when error < epsilon or after some iteration number
	for(int i = 1; i <= mIter; ++i){
		for(int j = 0; j < control_index.size(); ++j){
			double error = iterate(control_index[j], i);
			if(error < epsilon) break;
		}
	}
	
	//Just call mOldPose after this operation will brings the output 
}




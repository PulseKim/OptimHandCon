#include "Dynamics.hpp"
#include <math.h>



double T = 1.0;
double step;
int n;
int target_frame;
Eigen::VectorXd p;
Eigen::VectorXd v;
Eigen::VectorXd p_ball;
Eigen::VectorXd v_ball;


Dynamics::Dynamics(const WorldPtr& world, const Eigen::Vector3d& v_in): mWorld(world), v_target(v_in)
{
	std::vector<Tendon*> tempTendon;
	mController = dart::common::make_unique<Controller>(
		mWorld->getSkeleton("hand"),tempTendon);
	mBall = mWorld->getSkeleton("ball");
	mCharacter = mWorld->getSkeleton("hand");	
	step = mWorld->getTimeStep();
	n = T/step;
	target_frame = n/30;
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
	// std::cout << "target" <<std::endl;		
	for(int i = 0; i< n ; ++i){
		Eigen::VectorXd tempPose = mOldPose[i];
		double current_t = step * i;
		tempPose[index] = 0;
		int m = controlPts.size();
		for(int j =0; j < m; ++j){
			tempPose[index] += controlPts[j]*combination(m-1,j) * pow((1-current_t), (m-1-j)) * pow(current_t, j); 
		}
		target_positions.push_back(tempPose);
		
		// std::cout << tempPose[index]<<std::endl;
	}
	return target_positions;
}

double Dynamics::evaluateEnergy(std::vector<Eigen::VectorXd> target_pose)
{	
	
	for(int i = 0; i< n ; ++i){
		mController-> setTargetPosition(target_pose[i]);
		mController->clearForces();
		mController->addSPDForces();
		mWorld->step();
	}
	Eigen::Vector3d v_ball = mBall->getCOMLinearVelocity();
	double error = 0.0;
	double weight = 3.0;
	for(int i= 0; i<3; ++i)
		if(i ==1)
			error += (v_target[i] - v_ball[i]) * (v_target[i] - v_ball[i]);
		else
			error += weight * (v_target[i] - v_ball[i]) * (v_target[i] - v_ball[i]);

	// std::cout <<"velocity" << std::endl;
	// std::cout << v_ball << std::endl;
	// std::cout << error<<std::endl;
	return error;
}

double Dynamics::iterate(int index, int iter)
{
	
	std::cout << mControlPts[index] <<std::endl;

	//Setting Control points
	Eigen::VectorXd controlPts = mControlPts[index];
	Eigen::VectorXd controlPts_plus_eps = controlPts, controlPts_minus_eps = controlPts;
	Eigen::VectorXd grad = Eigen::VectorXd::Zero(controlPts.rows());
	double epslion = 0.005;
	double lambda = 0.0002/ sqrt(iter);

	double total_error = 0.0;

	//Calculate gradient
	for(int i = 0; i < controlPts_plus_eps.rows(); ++i){
		
		controlPts_plus_eps = controlPts;
		controlPts_plus_eps = controlPts;
		controlPts_plus_eps[i] += epslion;
		controlPts_minus_eps[i] -= epslion;
		
		std::vector<Eigen::VectorXd> target_plus = computePose(controlPts_plus_eps, index);
		std::vector<Eigen::VectorXd> target_minus = computePose(controlPts_minus_eps, index);
		
		initPose(target_plus[0]);
		double e_plus = evaluateEnergy(target_plus);
		mWorld->reset();
		mCharacter->setPositions(p);
		mCharacter->setVelocities(v);
		mBall->setPositions(p_ball);
		mBall->setVelocities(v_ball);

		initPose(target_minus[0]);
		double e_minus = evaluateEnergy(target_minus);
		mWorld->reset();
		mCharacter->setPositions(p);
		mCharacter->setVelocities(v);
		mBall->setPositions(p_ball);
		mBall->setVelocities(v_ball);

		total_error += e_minus;
		total_error += e_plus;
		grad[i] = (e_plus - e_minus)/(epslion * 2) * lambda;
		// std::cout << e_plus << "  "  << e_minus <<std::endl;
	}

	//Set control points and oldpositions
	mControlPts[index] = (mControlPts[index] - grad);
	for(int i= 0 ; i < mControlPts[index].size();++i)
	{
		if(mControlPts[index][i] > M_PI) mControlPts[index][i] = M_PI;
		else if(mControlPts[index][i] < -M_PI) mControlPts[index][i] =  - M_PI;
	}
	mOldPose = computePose(mControlPts[index], index);

	return total_error / 2;
}

//Pre-grabbing and move to the starting point
void Dynamics::initPose(Eigen::VectorXd initialPose)
{
	Eigen::VectorXd current = mCharacter->getPositions();
	Eigen::VectorXd grabbed = mController->grabOrOpen(current,true);
	Eigen::VectorXd pose = Eigen::VectorXd::Zero(mCharacter->getPositions().size());

	//Grabbing
	double preTime = 1.0;
	int preN = preTime / step;

	for(int i = 1; i < preN+1 ; ++i){		
		for(int j = 0; j < pose.size() ; ++j){
			pose[j] = current[j] + (grabbed[j] - current[j]) * i / preN;
		}
		mController->clearForces();
		mController->addSPDForces();
		mController->setTargetPosition(pose);
		mWorld->step();
		// std::cout << mBall->getCOMLinearVelocity() << std::endl;
	}
	
	//Go to first position step by step

	for(int i = 1; i < preN+1 ; ++i){
		
		for(int j = 0; j < pose.size() ; ++j){
			pose[j] = grabbed[j] + (initialPose[j] - grabbed[j]) * i / preN;
		}
		mController->clearForces();
		mController->addSPDForces();
		mController->setTargetPosition(pose);
		mWorld->step();
	}

	//Wait for stable status
	for(int i = 0; i < preN; ++i){
		mController->clearForces();
		mController->addSPDForces();
		mWorld->step();
	}

}

void Dynamics::optimize()
{

	//Values saved to be initialized
	p = mCharacter->getPositions();
	v = mCharacter->getVelocities();
	p_ball = mBall->getPositions();
	v_ball = mBall->getVelocities();

	//Initialize mControl, mOld, control indexes
	std::vector<int> control_index;
	control_index.push_back(2);

	//control_index.push_back(4);
	//control_index.push_back(5);

	Eigen::VectorXd pose = mCharacter->getPositions();
	Eigen::VectorXd grab_pose = mController->grabOrOpen(pose, true);

	int numb_control = 6;
	mControlPts.clear();
	for(int i = 0; i < pose.size(); ++i)
		mControlPts.push_back(Eigen::VectorXd::Zero(numb_control));

	int lambda = n * 85/100;
	//Initializing grabbing ==> Now, for simplicity, just given timing lamda as opener
	for(int i = 0; i < lambda ; ++i)
		mOldPose.push_back(grab_pose);
	for(int i = lambda; i < n; ++i){
		Eigen::VectorXd interpolate_pose = pose;
		if(i < lambda + 5)
			for(int j = 0; j < pose.size(); ++j)
				interpolate_pose[j] = pose[j] + (grab_pose[j] - pose[j]) * (lambda+4-i) / 4;
		else
			interpolate_pose = pose;

		mOldPose.push_back(interpolate_pose);
	}

	//Initial controlpoints
		for(int i = 0; i < numb_control; ++i){
			mControlPts[2][i] = pose[2] - 45 * M_PI/180 *(numb_control-1-i)/(numb_control-1);
		}

	//Initializing old pose
		for(int i = 0; i < control_index.size(); ++i){
			Eigen::VectorXd currentControl = mControlPts[control_index[i]];
			mOldPose = computePose(currentControl, control_index[i]);
		}

	//Parameters
		double epsilon = 0.01;
		int mIter = 1000;

	//Iterate for each control indexes
	//Stop when error < epsilon or after some iteration number
		for(int i = 1; i <= mIter; ++i){
			std::cout << i <<"th iteration" <<std::endl;
			for(int j = 0; j < control_index.size(); ++j){
				double error = iterate(control_index[j], i);
				std::cout << error << std::endl;
				if(error < epsilon) break;
			}	
		}


		initPose(mOldPose[0]);
		std::cout << evaluateEnergy(mOldPose) <<std::endl;
		mWorld->reset();
		mCharacter->setPositions(p);
		mCharacter->setVelocities(v);
		mBall->setPositions(p_ball);
		mBall->setVelocities(v_ball);
	//Just call mOldPose after this operation will brings the output 
	}

	std::vector<Eigen::VectorXd> Dynamics::poseGetter()
	{
		return mOldPose;
	}



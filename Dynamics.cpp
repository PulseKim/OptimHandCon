#include "Dynamics.hpp"
#include <math.h>



double T = 0.2;
double preTime = 0.2;
double step;
int n;
int target_frame;
double prev_error;
int local = 0;
double time_max;
double weight = 25.00;
Eigen::VectorXd p;
Eigen::VectorXd p_ball;

std::chrono::time_point<std::chrono::system_clock> time_check_s = std::chrono::system_clock::now();

void time_check_start()
{
	time_check_s = std::chrono::system_clock::now();
}

void time_check_end()
{
	std::chrono::duration<double> elapsed_seconds;
	elapsed_seconds = std::chrono::system_clock::now()-time_check_s;
	std::cout<<"time elapsed: " << elapsed_seconds.count()<<std::endl;
}

Dynamics::Dynamics(const WorldPtr& world, const Eigen::Vector3d& v_in):mOriginalWorld(world), v_target(v_in)
{
	this->init(mOriginalWorld);
	step = mWorld->getTimeStep();
	n = T/step;
	target_frame = n/30;

}

void Dynamics::init(const WorldPtr& world)
{
	mWorld = world->clone();
	std::vector<Tendon*> tempTendon;
	mController = dart::common::make_unique<Controller>(
		mWorld->getSkeleton("hand"),tempTendon);
	mBall = mWorld->getSkeleton("ball");
	mCharacter = mWorld->getSkeleton("hand");	
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
		double current_t = step * i/T;
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
		mController->clearForces();
		mController->addSPDForces();
		mController-> setTargetPosition(target_pose[i]);
		mWorld->step();
	}
	Eigen::Vector3d v_ball = mBall->getCOMLinearVelocity();
	double error = 0.0;
	for(int i= 0; i<3; ++i)
		if(i ==1){
			error += (v_target[i] - v_ball[i]) * (v_target[i] - v_ball[i]);
			// std::cout << "vel: " << v_ball[i] << std::endl;
		}
		else
			error += weight * (v_target[i] - v_ball[i]) * (v_target[i] - v_ball[i]);
	return error;
}

double Dynamics::evaluateEnergyPosition(std::vector<Eigen::VectorXd> target_pose)
{	
	Eigen::Vector3d max_real_pose;
	Eigen::Vector3d prev_real_pose;
	double epsilon = 0.03;
	Eigen::Vector3d current_real_pose;

	int i;
	for(i = 0; i< time_max * 10/ step + n; ++i){
		mController->clearForces();
		mController->addSPDForces();
		if(i < n)
			mController-> setTargetPosition(target_pose[i]);
		current_real_pose = mBall->getCOM();
		// std::cout << "this" << current_real_pose[1] <<std::endl;
		if(i > n && std::abs(mBall->getCOMLinearVelocity()[1]) < epsilon && current_real_pose[1] < prev_real_pose[1])
		{
			max_real_pose = prev_real_pose;
			break;
		}
		else
			prev_real_pose = current_real_pose;
		mWorld->step();
	}
	if(i == time_max * 4/ step + n) max_real_pose = prev_real_pose;

	// std::cout << "height : " << max_real_pose[1] << std::endl;

	if(max_real_pose[1] < 0) max_real_pose[1] = 0.0000001;
	double t_max = std::sqrt(2 * (max_real_pose[1]- p_ball[1])* std::abs(mWorld->getGravity()[1])) / std::abs(mWorld->getGravity()[1]);
	Eigen::Vector3d v_ball;
	v_ball[1] = std::sqrt(2 * (max_real_pose[1]- p_ball[4])* std::abs(mWorld->getGravity()[1]));
	v_ball[0] = (max_real_pose[0]-p_ball[3]) / t_max;
	v_ball[2] = (max_real_pose[2]-p_ball[5]) / t_max;

	std::cout << "vel_ball" << std::endl;
	std::cout << v_ball.transpose() <<std::endl;
	double error = 0.0;
	for(int i= 0; i<3; ++i)
		if(i ==1){
			error += (v_target[i] - v_ball[i]) * (v_target[i] - v_ball[i]);
			// std::cout << "vel: " << v_ball[i] << std::endl;
		}
		else
			error += weight * (v_target[i] - v_ball[i]) * (v_target[i] - v_ball[i]);
	// for(int i = 0; i < 3; ++i)
	// {
	// 	if(i ==1){
	// 		error += (max_pose_target[i] - max_real_pose[i]) * (max_pose_target[i] - max_real_pose[i]);
	// 	}
	// 	else
	// 		error += weight * (max_pose_target[i] - max_real_pose[i]) * (max_pose_target[i] - max_real_pose[i]);
	// }

	return error;
}

double Dynamics::radian(double angle){
	double rad = angle * M_PI / 180;
	return rad;
}


//Pre-grabbing and move to the starting point
void Dynamics::initPose(Eigen::VectorXd initialPose)
{

	this->init(mOriginalWorld);
	// mWorld->reset();
	// mCharacter->setPositions(p);
	// mCharacter->setVelocities(v);
	// mBall->setPositions(p_ball);
	// mBall->setVelocities(v_ball);


	Eigen::VectorXd current = mCharacter->getPositions();
	Eigen::VectorXd grabbed = mController->grabOrOpen(current,true);
	Eigen::VectorXd pose = Eigen::VectorXd::Zero(mCharacter->getPositions().size());

	//Grabbing
	int preN = preTime / step;

	// for(int i = 1; i < preN+1 ; ++i){		
	// 	for(int j = 0; j < pose.size() ; ++j){
	// 		pose[j] = current[j] + (grabbed[j] - current[j]) * i / preN;
	// 	}
	// 	mController->clearForces();
	// 	mController->addSPDForces();
	// 	mController->setTargetPosition(pose);
	// 	mWorld->step();
	// 	// std::cout << mBall->getCOMLinearVelocity() << std::endl;
	// }

	// //Wait for stable status
	// for(int i = 0; i < preN; ++i){
	// 	mController->clearForces();
	// 	mController->addSPDForces();
	// 	mWorld->step();
	// }
	// //Go to first position step by step



	Eigen::VectorXd pose_init = mCharacter->getPositions();
	for(int i = 0 ; i< 4 ; ++i){
		pose_init[i*4 + 7] = radian(66.0);
		pose_init[i*4 + 8] = radian(43.0);
		pose_init[i*4 + 9] = radian(58.0);
	}
	pose_init[22] = radian(90.0);
	pose_init[23] = radian(-40.0);
	pose_init[24] = radian(30.0);
	pose_init[25] = radian(40.0);
	pose_init[26] = radian(30.0);
	mCharacter->setPositions(pose_init);

	for(int i = 1; i < preN+1 ; ++i){
		mController->clearForces();
		mController->addSPDForces();
		for(int j = 0; j < pose.size() ; ++j){
			pose[j] = grabbed[j] + (initialPose[j] - grabbed[j]) * i / preN;
		}
		mController->setTargetPosition(pose);
		mWorld->step();
	}

	// 	//Wait for stable status
	// for(int i = 0; i < preN; ++i){
	// 	mController->clearForces();
	// 	mController->addSPDForces();
	// 	mWorld->step();
	// }
	// std::cout <<"Ball speed" << mBall->getCOM().transpose() << std::endl;

	
}

double Dynamics::GDiterate(int index, int iter)
{
	
	// std::cout << mControlPts[index] <<std::endl;

	//Setting Control points
	Eigen::VectorXd controlPts = mControlPts[index];
	Eigen::VectorXd controlPts_plus_eps = controlPts, controlPts_minus_eps = controlPts;
	Eigen::VectorXd grad = Eigen::VectorXd::Zero(controlPts.rows());
	double epsilon = 0.008;
	double lambda = 0.05;

	double total_error = 0.0;
	double max_angle_step = 0.4;

	//Calculate gradient
	for(int i = 0; i < controlPts_plus_eps.rows(); ++i){
		
		controlPts_plus_eps = controlPts;
		controlPts_minus_eps = controlPts;
		controlPts_plus_eps[i] += epsilon;
		controlPts_minus_eps[i] -= epsilon;

		std::vector<Eigen::VectorXd> target_plus = computePose(controlPts_plus_eps, index);
		std::vector<Eigen::VectorXd> target_minus = computePose(controlPts_minus_eps, index);
		
		// std::cout << "init " << std::endl;
		// time_check_start();
		initPose(target_plus[0]);
		// time_check_end();
		// std::cout << "energy" <<std::endl;
		// time_check_start();
		double e_plus = evaluateEnergy(target_plus);
		// time_check_end();

		initPose(target_minus[0]);
		double e_minus = evaluateEnergy(target_minus);

		total_error += std::abs(e_plus - e_minus);
		grad[i] = (e_plus - e_minus)/(epsilon * 2);
		// std::cout << e_plus << "  "  << e_minus <<std::endl;
	}

	//Set the value of lambda and control points and old positions
	int j;
	std::vector<Eigen::VectorXd> current_control = mControlPts;
	std::vector<Eigen::VectorXd> current_pose;
	int minIter = 10;
	for(j = 0; j < minIter ; j++)
	{
		Eigen::VectorXd computed_grad = Eigen::VectorXd::Zero(controlPts.rows());
		computed_grad = lambda * grad;
		for(int i =0 ; i < grad.rows(); ++i){
			if(computed_grad[i] > max_angle_step) computed_grad[i] = max_angle_step;
			if(computed_grad[i] < -max_angle_step) computed_grad[i] = -max_angle_step;
		}
		current_control[index] = (mControlPts[index] - computed_grad);
		for(int i= 0 ; i < current_control[index].size();++i)
		{
			if(current_control[index][i] > mCharacter->getDof(index)->getPositionUpperLimit()) current_control[index][i] = mCharacter->getDof(index)->getPositionUpperLimit();
			else if(current_control[index][i] < mCharacter->getDof(index)->getPositionLowerLimit()) current_control[index][i] =  mCharacter->getDof(index)->getPositionLowerLimit();
		}
		current_pose = computePose(current_control[index], index);

		initPose(current_pose[0]);
		double current_error = evaluateEnergy(current_pose);

		// std::cout << current_error <<std::endl;
		if(current_error < prev_error){
			prev_error = current_error;
			break;
		}
		lambda *= 0.5;
	}	
	

	if(j != minIter){
		mControlPts = current_control;
		mOldPose = current_pose;
	}
	else local++;
	return prev_error;
}

double Dynamics::GDiteratePose(int index, int iter)
{
	
	// std::cout << mControlPts[index] <<std::endl;

	//Setting Control points
	Eigen::VectorXd controlPts = mControlPts[index];
	Eigen::VectorXd controlPts_plus_eps = controlPts, controlPts_minus_eps = controlPts;
	Eigen::VectorXd grad = Eigen::VectorXd::Zero(controlPts.rows());
	double epsilon = 0.005;
	double lambda = 0.05/std::max(weight,0.9);

	double total_error = 0.0;
	double max_angle_step = 0.4;

	//Calculate gradient
	for(int i = 0; i < controlPts_plus_eps.rows(); ++i){
		
		controlPts_plus_eps = controlPts;
		controlPts_minus_eps = controlPts;
		controlPts_plus_eps[i] += epsilon;
		controlPts_minus_eps[i] -= epsilon;

		std::vector<Eigen::VectorXd> target_plus = computePose(controlPts_plus_eps, index);
		std::vector<Eigen::VectorXd> target_minus = computePose(controlPts_minus_eps, index);
		
		// std::cout << "init " << std::endl;
		// time_check_start();
		initPose(target_plus[0]);
		// time_check_end();
		// std::cout << "energy" <<std::endl;
		// time_check_start();
		double e_plus = evaluateEnergyPosition(target_plus);
		// time_check_end();
		initPose(target_minus[0]);
		double e_minus = evaluateEnergyPosition(target_minus);

		total_error += std::abs(e_plus - e_minus);
		grad[i] = (e_plus - e_minus)/(epsilon * 2);
		// std::cout << e_plus << "  "  << e_minus <<std::endl;
	}

	//Set the value of lambda and control points and old positions
	int j;
	std::vector<Eigen::VectorXd> current_control = mControlPts;
	std::vector<Eigen::VectorXd> current_pose;
	int minIter = 10;
	for(j = 0; j < minIter ; j++)
	{
		Eigen::VectorXd computed_grad = Eigen::VectorXd::Zero(controlPts.rows());
		computed_grad = lambda * grad;
		for(int i =0 ; i < grad.rows(); ++i){
			if(computed_grad[i] > max_angle_step) computed_grad[i] = max_angle_step;
			if(computed_grad[i] < -max_angle_step) computed_grad[i] = -max_angle_step;
		}
		current_control[index] = (mControlPts[index] - computed_grad);
		if(index == 2){
			for(int i= 0 ; i < current_control[index].size();++i)
			{
				if(current_control[index][i] > M_PI) current_control[index][i] = M_PI;
				else if(current_control[index][i] < -M_PI) current_control[index][i] =  -M_PI;
			}
		}
		if(index == 4){
			for(int i= 0 ; i < current_control[index].size();++i)
			{
				if(current_control[index][i] > M_PI * 120 / 180) current_control[index][i] = M_PI* 120 / 180;
				else if(current_control[index][i] < 0) current_control[index][i] =  0;
			}
		}
		current_pose = computePose(current_control[index], index);

		initPose(current_pose[0]);
		double current_error = evaluateEnergyPosition(current_pose);

		if(current_error < prev_error){
			prev_error = current_error;
			break;
		}
		lambda *= 0.5;
	}	
	

	if(j != minIter){
		mControlPts = current_control;
		mOldPose = current_pose;
		// std::cout << mOldPose[0][2] << std::endl;
		// std::cout << mOldPose[0][4] << std::endl;
		// std::cout << mOldPose[0][5] << std::endl;
	}
	else local++;
	return prev_error;
}



double Dynamics::SGDiterate(int index, int iter)
{
	
	std::cout << mControlPts[index] <<std::endl;

	//Setting Control points
	Eigen::VectorXd controlPts_plus_eps = mControlPts[index], controlPts_minus_eps = mControlPts[index];
	double epsilon = 0.008;
	double lambda = 0.0002/ sqrt(iter);

	double total_error = 0.0;
	double max_angle_step = 0.4;

	//Calculate gradient
	for(int i = 0; i < controlPts_plus_eps.rows(); ++i){
		
		controlPts_plus_eps = mControlPts[index];
		controlPts_minus_eps = mControlPts[index];
		controlPts_plus_eps[i] += epsilon;
		controlPts_minus_eps[i] -= epsilon;
		
		std::vector<Eigen::VectorXd> target_plus = computePose(controlPts_plus_eps, index);
		std::vector<Eigen::VectorXd> target_minus = computePose(controlPts_minus_eps, index);
		
		initPose(target_plus[0]);
		double e_plus = evaluateEnergy(target_plus);


		initPose(target_minus[0]);
		double e_minus = evaluateEnergy(target_minus);

		total_error += std::abs(e_plus - e_minus);
		double angle_step = (e_plus - e_minus)/(epsilon * 2) * lambda;
		if(angle_step > max_angle_step) angle_step = max_angle_step;
		if(angle_step < -max_angle_step) angle_step = -max_angle_step;

		mControlPts[index][i] -= angle_step;

		if(mControlPts[index][i] > M_PI) mControlPts[index][i] = M_PI;
		else if(mControlPts[index][i] < -M_PI) mControlPts[index][i] =  - M_PI;
	}

	mOldPose = computePose(mControlPts[index], index);
	return total_error / controlPts_plus_eps.rows();
}

int Dynamics::lambdaLearning(int iter, int lambda)
{
	Eigen::VectorXd grab_pose = mOldPose[0];
	int epsilon = 5;
	double weight = 2.0;
	int new_lambda = lambda;
	int lambda_plus = lambda + epsilon, lambda_minus = lambda - epsilon;
	double max_step = 20;

	if(lambda_plus > n-4){
		lambda_plus = n-4;
		lambda_minus = n-4 - epsilon;
	} 
	else if(lambda_minus < n * 0.5){
		lambda_minus = n * 0.5;
		lambda_plus = n * 0.5 +epsilon;
	}

	std::vector<Eigen::VectorXd> target_plus, target_minus;

	for(int i = 0; i < lambda_plus ; ++i){
		Eigen::VectorXd pose = mOldPose[i];
		for(int j = 6; j < grab_pose.size(); ++j)
			pose[j] = grab_pose[j];
		target_plus.push_back(pose);
	}

	for(int i = lambda_plus; i < n; ++i){
		Eigen::VectorXd interpolate_pose = mOldPose[i];
		if(i < lambda_plus + 5)
			for(int j = 6; j < p.size(); ++j)
				interpolate_pose[j] = p[j] + (grab_pose[j] - p[j]) * (lambda_plus+4-i) / 4;
		else
			interpolate_pose = p;
		target_plus.push_back(interpolate_pose);
	}
	for(int i = 0; i < lambda_minus ; ++i){
		Eigen::VectorXd pose = mOldPose[i];
		for(int j = 6; j < grab_pose.size(); ++j)
			pose[j] = grab_pose[j];
		target_minus.push_back(pose);
	}

	for(int i = lambda_minus; i < n; ++i){
		Eigen::VectorXd interpolate_pose = mOldPose[i];
		if(i < lambda_minus + 5)
			for(int j = 6; j < p.size(); ++j)
				interpolate_pose[j] = p[j] + (grab_pose[j] - p[j]) * (lambda_minus+4-i) / 4;
		else
			interpolate_pose = p;
		target_minus.push_back(interpolate_pose);
	}

	initPose(target_plus[0]);
	double e_plus = evaluateEnergy(target_plus);

	initPose(target_minus[0]);
	double e_minus = evaluateEnergy(target_minus);

	int step = (e_plus - e_minus) / (2 * epsilon) * weight;
	if(step > max_step) step = max_step;
	if(step < -max_step) step = -max_step;
	new_lambda -= step;
	if(new_lambda > n -4) new_lambda = n - 4;
	else if ( new_lambda < n * 0.5) new_lambda = n * 0.5;

	for(int i = 0; i < new_lambda; ++i){
		Eigen::VectorXd pose = mOldPose[i];
		for(int j = 6; j < grab_pose.size(); ++j)
			pose[j] = grab_pose[j];
		mOldPose[i] = pose;
	}

	for(int i = new_lambda; i < n; ++i){
		Eigen::VectorXd interpolate_pose = mOldPose[i];
		if(i < new_lambda + 5)
			for(int j = 6; j < p.size(); ++j)
				interpolate_pose[j] = p[j] + (grab_pose[j] - p[j]) * (new_lambda+4-i) / 4;
		else
			interpolate_pose = p;
		mOldPose[i] = interpolate_pose;
	}
	std::cout << "lambda learning error : ";
	std::cout << (e_plus + e_minus) / 2 << std::endl;
	std::cout << "new lambda is " << new_lambda << std::endl;
	return new_lambda;
}


void Dynamics::optimize(std::string name)
{
	//Delete collision aspects
	// deleteCollisionAspect();

	//Values saved to be initialized
	p = mCharacter->getPositions();
	p_ball = mBall->getPositions();

	for(int i = 0; i < 3; ++i)
		max_pose_target[i] = p_ball[i+3];
	double acc = mWorld->getGravity()[1];
	time_max = - v_target[1] / acc;
	for(int i = 0; i < 3; ++i){
		if(i ==1) max_pose_target[i] += (v_target[i]*time_max + 1.0 / 2.0 * acc * time_max* time_max);
		else max_pose_target[i] += v_target[i]*time_max;
	}
	std::cout<< "max_pose " <<std::endl;
	std::cout << max_pose_target.transpose() << std::endl;
	//Initialize mControl, mOld, control indexes
	std::vector<int> control_index;
	control_index.push_back(2);
	control_index.push_back(4);
	control_index.push_back(5);

	Eigen::VectorXd pose = mCharacter->getPositions();
	Eigen::VectorXd grab_pose = mController->grabOrOpen(pose, true);

	int numb_control = 3;
	mControlPts.clear();
	for(int i = 0; i < pose.size(); ++i)
		mControlPts.push_back(Eigen::VectorXd::Zero(numb_control));

	int lambda = n * 90/100;
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
		mControlPts[2][i] = pose[2] - 70 * M_PI/180 *(numb_control-1-i)/(numb_control-1);
		mControlPts[4][i] = pose[4] + 10 * M_PI/180 * i/(numb_control-1);
		mControlPts[5][i] = pose[5] - 40 * M_PI/180 *(numb_control-1-i)/(numb_control-1);
	}
	//Initializing old pose
	for(int i = 0; i < control_index.size(); ++i){
		Eigen::VectorXd currentControl = mControlPts[control_index[i]];
		mOldPose = computePose(currentControl, control_index[i]);
	}

	//Parameters
	double epsilon = 0.001;
	int mIter = 1000;

	initPose(mOldPose[0]);
	// prev_error = evaluateEnergy(mOldPose);
	prev_error = evaluateEnergyPosition(mOldPose);

	//Iterate for each control indexes
	//Stop when error < epsilon or after some iteration number
	for(int i = 1; i <= mIter; ++i){
		local = 0;
		// time_check_start();
		std::cout << i <<"th iteration" <<std::endl;
		double error = 0.0;
		for(int j = 0; j < control_index.size(); ++j){
			error = GDiteratePose(control_index[j], i);
			std::cout << "error is " << error << std::endl;
			if(error <epsilon) {
				local = control_index.size();
				break;
			}
		}
		// std::cout << "average grad mag : " << error << std::endl;
		if(local == control_index.size()) break;
		// time_check_end();
		// lambda = lambdaLearning(i, lambda);
	}

	std::cout << "mini simulation" << std::endl;

	// mCharacter->resetCommands();
	// mBall->resetCommands();
	initPose(mOldPose[0]);
	double final_error = evaluateEnergyPosition(mOldPose);
	std::cout << "final error1 " << final_error <<std::endl;

	initPose(mOldPose[0]);
	final_error = evaluateEnergyPosition(mOldPose);
	std::cout << "final error2 " << final_error <<std::endl;

	this->init(mOriginalWorld);

	//Save algorithm
	std::ofstream outFile("Result" + name + ".txt");	
	for(int i = 0; i < control_index.size();++i){
		// std::cout << "result control point "<< control_index[i] << "th" <<std::endl;
		outFile << "The result control point"<< control_index[i] << "is " << std::endl;
		// std::cout << mControlPts[control_index[i]] << std::endl;
		outFile << mControlPts[control_index[i]].transpose() << std::endl;
	}
	// addCollisionAspect();
	outFile << "The error is " << std::endl;
	outFile << prev_error << std::endl;
	outFile.close();

}

std::vector<Eigen::VectorXd> Dynamics::poseGetter()
{
	return mOldPose;
}


Eigen::Vector3d Dynamics::targetGetter()
{
	return max_pose_target;
}

void Dynamics::deleteCollisionAspect()
{
	bool flag = false;
	BodyNode* bn = mCharacter->getBodyNode("root");
	auto colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("arm_ball");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("arm_univ");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("thumb_ball");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("thumb_revol_down");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);

	for(int idx = 0 ; idx < 4 ; ++idx)
	{
		bn = mCharacter->getBodyNode("weld" + std::to_string(idx));
		colNodes = bn->getShapeNodesWith<CollisionAspect>();
		colNodes[0]->getCollisionAspect()->setCollidable(flag);
		bn = mCharacter->getBodyNode("univ" + std::to_string(idx));
		colNodes = bn->getShapeNodesWith<CollisionAspect>();
		colNodes[0]->getCollisionAspect()->setCollidable(flag);
		bn = mCharacter->getBodyNode("revol_down" + std::to_string(idx));
		colNodes = bn->getShapeNodesWith<CollisionAspect>();
		colNodes[0]->getCollisionAspect()->setCollidable(flag);
	}
}
void Dynamics::addCollisionAspect()
{
	bool flag = true;
	BodyNode* bn = mCharacter->getBodyNode("root");
	auto colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("arm_ball");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("arm_univ");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("thumb_ball");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);
	bn = mCharacter->getBodyNode("thumb_revol_down");
	colNodes = bn->getShapeNodesWith<CollisionAspect>();
	colNodes[0]->getCollisionAspect()->setCollidable(flag);

	for(int idx = 0 ; idx < 4 ; ++idx)
	{
		bn = mCharacter->getBodyNode("weld" + std::to_string(idx));
		colNodes = bn->getShapeNodesWith<CollisionAspect>();
		colNodes[0]->getCollisionAspect()->setCollidable(flag);
		bn = mCharacter->getBodyNode("univ" + std::to_string(idx));
		colNodes = bn->getShapeNodesWith<CollisionAspect>();
		colNodes[0]->getCollisionAspect()->setCollidable(flag);
		bn = mCharacter->getBodyNode("revol_down" + std::to_string(idx));
		colNodes = bn->getShapeNodesWith<CollisionAspect>();
		colNodes[0]->getCollisionAspect()->setCollidable(flag);
	}
}
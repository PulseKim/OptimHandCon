#include "IkSolver.hpp"
#include <iostream>


IkSolver::IkSolver(){

}

Eigen::VectorXd IkSolver::TotalIk(std::vector<std::pair<Eigen::Vector3d, int>> Ends, const SkeletonPtr&hand){
	Eigen::VectorXd newPose = hand->getPositions();
	//std::cout << newPose;
	for(int j=0 ; j < Ends.size(); ++j){
		newPose += IKSingleConfig(Ends[j].first, hand, Ends[j].second);
	}

	return newPose;
}

Eigen::VectorXd IkSolver::IKMiddle(const Eigen::Vector3d target, const SkeletonPtr& hand, std::string name)
{
	BodyNode* currentFinger = hand->getBodyNode(name);
	Eigen::Vector3d deviation = target- currentFinger->getCOM();
	Eigen::MatrixXd J = hand->getLinearJacobian(currentFinger);
	Eigen::MatrixXd JJT = J*J.transpose();
	Eigen::MatrixXd pseudoJ = J.transpose() * (JJT+ 0.0025 * Eigen::MatrixXd::Identity(JJT.rows(), JJT.cols())).inverse();
	//std::cout << pseudoJ * deviation << std::endl;
	Eigen::VectorXd newPose = pseudoJ * deviation;
	newPose = newPose * 0.3;

	return newPose;
}

Eigen::VectorXd IkSolver::IKSingleConfig(const Eigen::Vector3d endEff, const SkeletonPtr& hand, int idx)
{
	BodyNode* currentFinger;
	if(idx <4) currentFinger = hand->getBodyNode("revol_up" + std::to_string(idx));
	else currentFinger = hand->getBodyNode("thumb_revol_up");
	Eigen::Vector3d deviation = endEff- currentFinger->getCOM();

	Eigen::MatrixXd J = hand->getLinearJacobian(currentFinger);

	//Weight Parameterize for arm and palm
	for(int i = 0; i <6 ; ++i){
		J.col(i) = J.col(i) * 0.4;
	}

	//Weight Parameterize for fingers
	for(int i = 0; i <5 ; ++i){
		J.col(4*i+6) = J.col(4*i+6) * 0.1;
		J.col(4*i+7) = J.col(4*i+7) * 1.5;
		J.col(4*i+8) = J.col(4*i+8) * 1.5;
		J.col(4*i+9) = J.col(4*i+9) * 1.8;
	}

	Eigen::MatrixXd JJT = J*J.transpose();
	Eigen::MatrixXd pseudoJ = J.transpose() * (JJT+ 0.0025 * Eigen::MatrixXd::Identity(JJT.rows(), JJT.cols())).inverse();
	//std::cout << pseudoJ * deviation << std::endl;
	Eigen::VectorXd newPose = pseudoJ * deviation;
	newPose = newPose * 0.5;

	return newPose;
}


Eigen::VectorXd IkSolver::IKMultiple(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector3d, int>> Ends, int iter)
{	
	Eigen::MatrixXd J_stack = Eigen::MatrixXd::Zero(Ends[0].first.rows()*Ends.size(),hand->getPositions().size());
	Eigen::VectorXd dev_stack;
	dev_stack = Eigen::VectorXd::Zero(Ends.size()*3); 

	for(int i = 0 ; i < Ends.size() ; i ++){
		BodyNode* currentFinger;
		if(Ends[i].second !=4) currentFinger = hand->getBodyNode("revol_up" + std::to_string(Ends[i].second));
		else currentFinger = hand->getBodyNode("thumb_revol_up");
		Eigen::Vector3d deviation = Ends[i].first - currentFinger->getCOM();
		Eigen::MatrixXd J = hand->getLinearJacobian(currentFinger);
		//Weight Parameterize for arm and palm
		for(int i = 0; i <6 ; ++i){
			J.col(i) = J.col(i) * (2 *(iter+2)/grad_Iter);
		}

		//Weight Parameterize for fingers
		for(int i = 0; i <5 ; ++i){
			for(int j = 0; j <4; ++j)
				J.col(4*i+6+j) = J.col(4*i+6+j) * (2 * (grad_Iter-iter+2)/grad_Iter);
		}

		J.col(hand->getPositions().size()-1) = J.col(hand->getPositions().size()-1) * (2.3*(grad_Iter-iter+2)/grad_Iter);

		J_stack.block(J.rows()*i,0,J.rows(),J.cols()) = J;
		dev_stack(3*i) = deviation[0];
		dev_stack(3*i+1) = deviation[1];
		dev_stack(3*i+2) = deviation[2];
	}
	Eigen::MatrixXd JJT = J_stack*J_stack.transpose();
	Eigen::MatrixXd pseudoJ = J_stack.transpose() * (JJT+ 0.0025 * Eigen::MatrixXd::Identity(JJT.rows(), JJT.cols())).inverse();

	Eigen::VectorXd newPose = pseudoJ * dev_stack;
	newPose = newPose * 0.15;
	return newPose;
}

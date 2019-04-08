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

Eigen::VectorXd IkSolver::IKSingleConfig(const Eigen::Vector3d endEff, const SkeletonPtr& hand, int idx)
{
	BodyNode* currentFinger = hand->getBodyNode("revol_up" + std::to_string(idx));
	Eigen::Vector3d deviation = endEff- currentFinger->getCOM();

	// Eigen::MatrixXd J = hand->getLinearJacobian(currentFinger, rootFinger);
	Eigen::MatrixXd J = hand->getLinearJacobian(currentFinger);

	// for(int i=0 ; i < 6 ; ++i){
	// 		J.col(i) = Eigen::Vector3d(0, 0, 0);
	// }
	for(int i=0 ; i < 4 ; ++i){
		J.col(5 * i + 7) = Eigen::Vector3d(0, 0, 0);
		//J.col(5 * i + 6) = Eigen::Vector3d(0, 0, 0);
	}

	
	Eigen::MatrixXd pseudoJ = J.transpose() * (J*J.transpose()).inverse();
	//std::cout << pseudoJ * deviation << std::endl;
	Eigen::VectorXd newPose = pseudoJ * deviation;
	newPose = newPose * 0.5;

	return newPose;
}

#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
using namespace dart::common;
using namespace dart::dynamics;

class IkSolver{
public: 
	IkSolver();

	Eigen::VectorXd IKSingleConfig(const Eigen::Vector3d endEff, const SkeletonPtr& hand, int idx);
	Eigen::VectorXd TotalIk(std::vector<std::pair<Eigen::Vector3d, int>> Ends, const SkeletonPtr&hand);

};
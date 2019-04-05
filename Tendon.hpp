#ifndef __TENDON_H__
#define __TENDON_H__
#include <iostream>
#include "dart/dart.hpp"
#include <tuple>


class Tendon
{
public:
	Tendon(std::string _name);
	void AddAnchor(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& pos, bool isGlobal);
	void AddAnchor(dart::dynamics::BodyNode* bn, const Eigen::Vector3d& pos, const Eigen::Vector3d& dir_up, const Eigen::Vector3d& dir_down, bool isGlobal);	
	const std::vector<std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>>& GetAnchors(){return mAnchors;}
	void Update(double time_step);
	void ApplyForceToBody();
	void ApplyForceToBody(double force_ext);
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ApplyForceToBodyDir(double force_ext);



	int GetNumRelatedDofs(){return num_related_dofs;};
	Eigen::VectorXd GetRelatedJtA();
	Eigen::MatrixXd GetJacobianTranspose();
	Eigen::Vector3d GetPoint(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& local_pos);
	Eigen::Vector3d GetPoint(const std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>& bnpos);
	Eigen::Vector3d GetPoint(const std::tuple<dart::dynamics::BodyNode*, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>& bnpos);
	Eigen::Vector3d GetDir(const std::tuple<dart::dynamics::BodyNode*, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>& bnpos, int flag);


public:
	double pd_force;
	double GetForce();
public:
	std::string name;
	std::vector<std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>> mAnchors;
	std::vector<std::tuple<dart::dynamics::BodyNode*, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>mAnchor_dir;
	int num_related_dofs;
	std::vector<int> related_dof_indices;
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> currForces;
	double mForce;
};


#endif
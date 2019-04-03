#include "Tendon.hpp"
#include <dart/dart.hpp>

int cnt = 10;

Tendon::
Tendon(std::string _name)
:name(_name)
{
}

void Tendon::AddAnchor(dart::dynamics::BodyNode* bn, const Eigen::Vector3d& pos, bool isGlobal)
{
	Eigen::Vector3d local_pos;
	if(isGlobal)
		local_pos = bn->getTransform().inverse()*pos;
	else
		local_pos = pos;
	mAnchors.push_back(std::make_pair(bn,local_pos));
}


void Tendon::AddAnchor(dart::dynamics::BodyNode* bn, const Eigen::Vector3d& pos, const Eigen::Vector3d& dir_up, const Eigen::Vector3d& dir_down, bool isGlobal)
{
	Eigen::Vector3d local_pos;
	Eigen::Vector3d loc_dir_up;
	Eigen::Vector3d loc_dir_down;
	if(isGlobal){
		local_pos = bn->getTransform().inverse()*pos;
		loc_dir_up = bn->getTransform().inverse()* dir_up;
		loc_dir_down = bn->getTransform().inverse()* dir_down;
	}
	else {
		local_pos = pos;
		loc_dir_up = dir_up;
		loc_dir_down = dir_down;
	}
	mAnchor_dir.push_back(std::make_tuple(bn,local_pos,loc_dir_up, loc_dir_down));
}





void Tendon::ApplyForceToBody()
{
	double f = GetForce();

	std::vector<Eigen::Vector3d> point;
	for(int i =0;i<mAnchors.size();i++){
		point.push_back(GetPoint(mAnchors[i]));
		//std::cout << GetPoint(mAnchors[i]) <<std::endl;
	}
	for(int i =0;i<mAnchors.size()-1;i++)
	{
		Eigen::Vector3d dir = point[i+1]-point[i];
		dir.normalize();
		dir = f*dir;
		mAnchors[i].first->addExtForce(dir,mAnchors[i].second);
	}

	for(int i =1;i<mAnchors.size();i++)
	{
		Eigen::Vector3d dir = point[i-1]-point[i];
		dir.normalize();
		dir = f*dir;
		mAnchors[i].first->addExtForce(dir,mAnchors[i].second);
	}
}

void Tendon::ApplyForceToBody(double force_ext)
{
	//if(cnt > 0) std::cout<< "cnt" << cnt <<std::endl;

	std::vector<Eigen::Vector3d> point;
	for(int i =0;i<mAnchors.size();i++){
		point.push_back(GetPoint(mAnchors[i]));
		//std::cout << GetPoint(mAnchors[i]) <<std::endl;
	}
	for(int i =0;i<mAnchors.size()-1;i++)
	{
		Eigen::Vector3d dir = point[i+1]-point[i];
		if(dir != Eigen::Vector3d(0, 0, 0)) dir.normalize();
		//if(cnt > 0) std::cout <<"dir" << i << "th force" << dir << std::endl;

		dir = force_ext*dir;
		mAnchors[i].first->addExtForce(dir,mAnchors[i].second);
	}

	for(int i =1;i<mAnchors.size();i++)
	{
		Eigen::Vector3d dir = point[i-1]-point[i];
		if(dir != Eigen::Vector3d(0, 0, 0)) dir.normalize();
		//if(cnt > 0) std::cout <<"op" << i << "th force" << dir << std::endl;
		dir = force_ext*dir;
		mAnchors[i].first->addExtForce(dir,mAnchors[i].second);
	}
	cnt --;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
Tendon::ApplyForceToBodyDir(double force_ext)
{
	currForces.clear();
	for(std::size_t i = 0 ; i < mAnchor_dir.size()-1; i++){
		Eigen::Vector3d loc_dir = std::get<2>(mAnchor_dir[i]);
		loc_dir = std::get<0>(mAnchor_dir[i])->getTransform().linear()*loc_dir;
		loc_dir.normalize();
		//std::cout << loc_dir << std::endl;
		loc_dir = loc_dir * force_ext;
		currForces.push_back(std::make_pair(loc_dir,Tendon::GetPoint(mAnchor_dir[i])));
		std::get<0>(mAnchor_dir[i])->addExtForce(loc_dir,std::get<1>(mAnchor_dir[i]));
		// std::cout << "Upper force" << std::endl;
		// std::cout << loc_dir << std::endl;
	}
	for(std::size_t i = 1 ; i < mAnchor_dir.size(); i++){
		Eigen::Vector3d loc_dir = std::get<3>(mAnchor_dir[i]);
		loc_dir = std::get<0>(mAnchor_dir[i])->getTransform().linear()*loc_dir;
		loc_dir.normalize();
		loc_dir = loc_dir * force_ext;
		currForces.push_back(std::make_pair(loc_dir,Tendon::GetPoint(mAnchor_dir[i])));
		// if(cnt > 0)
		// {
		// 	//std::cout << i <<std::endl;
		// 	//std::cout << std::get<0>(mAnchor_dir[i])->getTransform().linear() << std::endl;
		// 	//std::cout << loc_dir << std::endl;
		// 	std::cout << Tendon::GetPoint(mAnchor_dir[i])  << std::endl;
		// 	std::cout << loc_dir << std::endl;
		// 	std::cout << r_vector.cross(loc_dir).norm() <<std::endl;
		// }
		std::get<0>(mAnchor_dir[i])->addExtForce(loc_dir,std::get<1>(mAnchor_dir[i]));
		// std::cout << "Lower force" << std::endl;
		// std::cout << loc_dir << std::endl;

	}
	cnt --;
	return currForces;
}




double Tendon::GetForce()
{
	return 500000.0;
	//return Getf_A()*activation + Getf_p();
}

Eigen::Vector3d GetPoint(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& local_pos)
{
	return bn->getTransform()*local_pos;
}

Eigen::Vector3d GetPoint(const std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>& bnpos)
{
	return bnpos.first->getTransform()*bnpos.second;
}


Eigen::MatrixXd Tendon::GetJacobianTranspose()
{
	const auto& skel = mAnchors[0].first->getSkeleton();
	int dof = skel->getNumDofs();
	Eigen::MatrixXd Jt(dof,3*mAnchors.size());

	Jt.setZero();
	for(int i =0;i<mAnchors.size();i++)
		Jt.block(0,i*3,dof,3) = skel->getLinearJacobian(mAnchors[i].first,mAnchors[i].second).transpose();
	
	return Jt;	
}



Eigen::Vector3d Tendon::GetPoint(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& local_pos)
{
	return bn->getTransform()*local_pos;
}
Eigen::Vector3d Tendon::GetPoint(const std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d>& bnpos)
{
	return bnpos.first->getTransform()*bnpos.second;
}
Eigen::Vector3d Tendon::GetPoint(const std::tuple<dart::dynamics::BodyNode*, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>& bnpos)
{
	return std::get<0>(bnpos)->getTransform() * std::get<1>(bnpos);
}
Eigen::Vector3d Tendon::GetDir(const std::tuple<dart::dynamics::BodyNode*, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>& bnpos, int flag)
{
	if(flag == 0) return std::get<0>(bnpos)->getTransform().linear() * std::get<2>(bnpos);
	else return std::get<0>(bnpos)->getTransform().linear()* std::get<3>(bnpos);
}


// std::pair<Eigen::VectorXd,Eigen::VectorXd> Tendon::GetForceJacobianAndPassive()
// {
// 	double f_a = Getf_A();
// 	double f_p = Getf_p();

// 	std::vector<Eigen::Vector3d> point,force_dir;
// 	for(int i =0;i<mAnchors.size();i++){
// 		point.push_back(GetPoint(mAnchors[i]));
// 		force_dir.push_back(Eigen::Vector3d::Zero());
// 	}
// 	for(int i =0;i<mAnchors.size()-1;i++)
// 	{
// 		Eigen::Vector3d dir = point[i+1]-point[i];
// 		dir.normalize();
// 		force_dir[i] += dir;
// 	}


// 	for(int i =1;i<mAnchors.size();i++)
// 	{
// 		Eigen::Vector3d dir = point[i-1]-point[i];
// 		dir.normalize();
// 		force_dir[i] += dir;
// 	}

// 	Eigen::VectorXd A(3*mAnchors.size());
// 	Eigen::VectorXd p(3*mAnchors.size());
// 	A.setZero();
// 	p.setZero();

// 	for(int i =0;i<mAnchors.size();i++)
// 	{
// 		A.segment<3>(i*3) = force_dir[i]*f_a;
// 		p.segment<3>(i*3) = force_dir[i]*f_p;
// 	}
// 	return std::make_pair(A,p);
// }

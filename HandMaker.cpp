#include "HandMaker.hpp"
#include <iostream>
#include <dart/dart.hpp>

HandMaker::HandMaker(){
	
	//makeFingers();
	//makeThum();
}
HandMaker::~HandMaker(){}


void HandMaker::makeHand(const SkeletonPtr& hand){
	mPalm = makePalm(hand);
	makeFingers(hand);
	makeThumb(hand);
	mSkel.setGeometry(hand, "ball0", 20.0, 2);
	mSkel.setGeometry(hand, "thumb_ball", 45.0, 0);
	mSkel.setGeometry(hand, "thumb_ball", -30.0, 1);
	mSkel.setGeometry(hand, "thumb_ball", 20.0, 2);

	for(std::size_t i = 0; i < hand->getNumJoints(); ++i)
		hand->getJoint(i)->setPositionLimitEnforced(true);

	for(int i = 0; i < 4; ++i)
		tendonSingleFinger(hand, i);
	tendonThumb(hand);
}

BodyNode* HandMaker::makePalm(const SkeletonPtr& hand)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(Eigen::Vector3d(hand_z, palm_height, palm_width)));
	double mass = default_mass;
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));
	BodyNode* bn;
	FreeJoint::Properties props;
	Eigen::Isometry3d T;
	T.setIdentity();
	T.translation() = Eigen::Vector3d(0.0,palm_height/2,hand_offset);
	props.mT_ParentBodyToJoint = T;
	bn = hand->createJointAndBodyNodePair<FreeJoint>(
		nullptr,props,BodyNode::AspectProperties("palm")).second;
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	bn->setInertia(inertia);
	return bn;	
}

void HandMaker::makeFingers(const SkeletonPtr& hand)
{
	for(int i = 0 ; i <4 ; ++i)
		makeSingleFinger(hand, i);
}

void HandMaker::makeSingleFinger(const SkeletonPtr& hand, int idx)
{
	double x_off = -palm_width / 2 + finger_width / 2 + (finger_width + gap) * idx;;
	std::string name = "weld" + std::to_string(idx);	
	BodyNode* bn = mSkel.makeWeldJoint(hand, mPalm, name, hand_z, weld_height, finger_width, 0.0, palm_height/2, x_off);
	name = "ball" + std::to_string(idx);	
	bn = mSkel.makeBallJoint(hand, bn, name, hand_z, finger_height, finger_width, 0.0, weld_height/2, 0.0);
	name = "revol_down" + std::to_string(idx);
	bn = mSkel.makeRevoluteJoint(hand, bn, name, hand_z, finger_height, finger_width, 0.0, finger_height/2, 0.0);
	name = "revol_up" + std::to_string(idx);
	bn = mSkel.makeRevoluteJoint(hand, bn, name, hand_z, finger_height, finger_width, 0.0, finger_height/2, 0.0);

}


void HandMaker::makeThumb(const SkeletonPtr& hand)
{
	std::string name = "thumb_weld";
	BodyNode* bn = mSkel.makeWeldJoint(hand, mPalm, name, hand_z, weld_height, thumb_width, 0.0, -thumb_height, palm_width/2);
	name = "thumb_ball";
	bn = mSkel.makeBallJoint(hand, bn, name, hand_z, thumb_height, thumb_width, 0.0, weld_height/2, 0.0);
	name = "thumb_revol_down";
	bn = mSkel.makeRevoluteJoint(hand, bn, name, hand_z, thumb_height, thumb_width, 0.0, thumb_height/2, 0.0);
	name = "thumb_revol_up";
	bn = mSkel.makeRevoluteJoint(hand, bn, name, hand_z, thumb_height, thumb_width, 0.0, thumb_height/2, 0.0);

}


void HandMaker::tendonSingleFinger(const SkeletonPtr& hand, int idx)
{
	std::string currName = "weld" + std::to_string(idx);
	std::string tendonName = std::to_string(idx) + "tendon longitude ";
	BodyNode* bn = hand->getBodyNode(currName);

	for(std::size_t i =0 ; i < 3; ++i){
		fingerTendon.push_back(std::make_pair(idx, new Tendon(tendonName + std::to_string(i) + "_ant")));
		fingerTendon.push_back(std::make_pair(idx, new Tendon(tendonName + std::to_string(i) + "_pos")));

	}
	Eigen::Vector3d down_link(finger_width/2, -finger_height/6, 0);
	Eigen::Vector3d down_opp(-finger_width/2, -finger_height/6, 0);
	Eigen::Vector3d down_dir(0, -finger_height/6, 0);
	Eigen::Vector3d upper_link(finger_width/2, weld_height/6, 0);
	Eigen::Vector3d upper_opp(-finger_width/2, weld_height/6, 0);
	Eigen::Vector3d upper_dir(0, finger_height/6, 0);
	Eigen::Vector3d null_dir(0,0,0);

	down_dir.normalize();
	upper_dir.normalize();

	std::size_t i;
	for(i = 0 ; i < 3; ++i){
		fingerTendon[2 * i + 8*idx].second-> AddAnchor(bn, upper_link, upper_dir, null_dir, false);
		fingerTendon[2 * i+1 + 8*idx].second-> AddAnchor(bn, upper_opp, upper_dir, null_dir, false);
		if(i == 0) {
			upper_link = Eigen::Vector3d(finger_width/2, finger_height/6, 0);
			upper_opp = Eigen::Vector3d(-finger_width/2, finger_height/6, 0);
		}
		bn = bn->getChildBodyNode(0);
		fingerTendon[2 * i + 8*idx].second-> AddAnchor(bn, down_link, null_dir, down_dir, false);
		fingerTendon[2 * i + 1 + 8*idx].second-> AddAnchor(bn, down_opp, null_dir, down_dir, false);
	}
	
	// axial joint control
	tendonName = std::to_string(idx) + "tendon axial ";
	bn = hand->getBodyNode(currName);
	fingerTendon.push_back(std::make_pair(idx, new Tendon(tendonName + "ant")));
	fingerTendon.push_back(std::make_pair(idx, new Tendon(tendonName + "pos")));

	down_link = Eigen::Vector3d(0, -finger_height/6, finger_width/2);
	down_opp = Eigen::Vector3d(0, -finger_height/6, -finger_width/2);
	upper_link = Eigen::Vector3d(0, weld_height/6, finger_width/2);
	upper_opp = Eigen::Vector3d(0, weld_height/6, -finger_width/2);


	fingerTendon[2 * i + 8*idx].second-> AddAnchor(bn, upper_link, upper_dir, null_dir, false);
	fingerTendon[2 * i+1 + 8*idx].second-> AddAnchor(bn, upper_opp, upper_dir, null_dir, false);
	bn = bn->getChildBodyNode(0);
	fingerTendon[2 * i + 8*idx].second-> AddAnchor(bn, down_link, null_dir, down_dir, false);
	fingerTendon[2 * i+1 + 8*idx].second-> AddAnchor(bn, down_opp, null_dir, down_dir, false);
}

void HandMaker::tendonThumb(const SkeletonPtr& hand){
	std::string currName = "thumb_weld";
	std::string tendonName = "thumb_longitude";

	BodyNode* bn = hand->getBodyNode(currName);

	for(std::size_t i = 0; i<3 ; ++i){
		fingerTendon.push_back(std::make_pair(4, new Tendon(tendonName + "ans")));
		fingerTendon.push_back(std::make_pair(4, new Tendon(tendonName + "pos")));
	}
	Eigen::Vector3d down_link(thumb_width/2, -thumb_height/6, 0);
	Eigen::Vector3d down_opp(-thumb_width/2, -thumb_height/6, 0);
	Eigen::Vector3d down_dir(0, -thumb_height/6, 0);
	Eigen::Vector3d upper_link(thumb_width/2, weld_height/6, 0);
	Eigen::Vector3d upper_opp(-thumb_width/2, weld_height/6, 0);
	Eigen::Vector3d upper_dir(0, thumb_height/6, 0);
	Eigen::Vector3d null_dir(0,0,0);

	down_dir.normalize();
	upper_dir.normalize();

	std::size_t i;
	for(i = 0 ; i < 3; ++i){
		fingerTendon[2 * i + 8*4].second-> AddAnchor(bn, upper_link, upper_dir, null_dir, false);
		fingerTendon[2 * i+1 + 8*4].second-> AddAnchor(bn, upper_opp, upper_dir, null_dir, false);
		if(i == 0) {
			upper_link = Eigen::Vector3d(thumb_width/2, thumb_height/6, 0);
			upper_opp = Eigen::Vector3d(-thumb_width/2, thumb_height/6, 0);
		}
		bn = bn->getChildBodyNode(0);
		fingerTendon[2 * i + 8*4].second-> AddAnchor(bn, down_link, null_dir, down_dir, false);
		fingerTendon[2 * i + 1 + 8*4].second-> AddAnchor(bn, down_opp, null_dir, down_dir, false);
	}

	tendonName = "thumb tendon axial ";
	bn = hand->getBodyNode(currName);
	fingerTendon.push_back(std::make_pair(4, new Tendon(tendonName + "ant")));
	fingerTendon.push_back(std::make_pair(4, new Tendon(tendonName + "pos")));

	down_link = Eigen::Vector3d(0, -thumb_height/6, thumb_width/2);
	down_opp = Eigen::Vector3d(0, -thumb_height/6, -thumb_width/2);
	upper_link = Eigen::Vector3d(0, weld_height/6, thumb_width/2);
	upper_opp = Eigen::Vector3d(0, weld_height/6, -thumb_width/2);


	fingerTendon[2 * i + 8*4].second-> AddAnchor(bn, upper_link, upper_dir, null_dir, false);
	fingerTendon[2 * i+1 + 8*4].second-> AddAnchor(bn, upper_opp, upper_dir, null_dir, false);
	bn = bn->getChildBodyNode(0);
	fingerTendon[2 * i + 8*4].second-> AddAnchor(bn, down_link, null_dir, down_dir, false);
	fingerTendon[2 * i+1 + 8*4].second-> AddAnchor(bn, down_opp, null_dir, down_dir, false);

}
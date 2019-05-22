#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "SkelParser.hpp"
#include "Tendon.hpp"

const double finger_width = 0.018;
const double finger_height = 0.032;
const double weld_height = 0.001;
const double gap = 0.002;
const double palm_width = 0.08;
const double palm_height = 0.1;
const double thumb_width = 0.02;
const double thumb_height = 0.03;
const double hand_z = 0.03;
const double arm_width = 0.065;
const double arm_height = 0.3;


class HandMaker{
public:
  /// Constructor
	HandMaker();
	~HandMaker();

	void makeHand(const SkeletonPtr& hand);
	BodyNode* makeArm(const SkeletonPtr& hand);
	void  makeFingers(const SkeletonPtr& hand);
	BodyNode* makePalm(const SkeletonPtr& hand, BodyNode* arm);
	void makeThumb(const SkeletonPtr& hand);
	void makeSingleFinger(const SkeletonPtr& hand, int idx);
	void tendonSingleFinger(const SkeletonPtr& hand, int idx);
	void tendonThumb(const SkeletonPtr& hand);


public:
	std::vector<std::pair<int, Tendon*>> fingerTendon;

protected:
	BodyNode* mPalm;
	SkeletonPtr mHand;
	SkelParser mSkel;

};
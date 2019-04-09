#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "SkelParser.hpp"
#include "Tendon.hpp"

const double finger_width = 0.5;
const double finger_height = 1.0;
const double weld_height = 0.2;
const double gap = 0.2;
const double palm_width = 2.6;
const double palm_height = 3.0;
const double thumb_width = 0.7;
const double thumb_height = 1.0;
const double hand_z = 0.5;
const double hand_offset = -2.0;
const double arm_width = 2.0;
const double arm_height =4.0;


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
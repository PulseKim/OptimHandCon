#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::math;

const double default_link_len = 2.0;
const double default_width = 0.5;
const double default_mass = 0.01;

class SkelParser{
public:
  /// Constructor
  SkelParser();

  ~SkelParser();
  
  void makeFloor(const SkeletonPtr& floor, const std::string& name);
  void makeFinger(const SkeletonPtr& skel);
  void sphereJointVis(const SkeletonPtr& skel, const SkeletonPtr& ball);
  BodyNode* makeRoot(const SkeletonPtr& skel, const std::string& name);
  BodyNode* makeBallJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name);
  BodyNode* makeBallJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, 
  double width, double length, double z_len, double x_offset, double y_offset, double z_offset);
  BodyNode* makeRevoluteJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name);
  BodyNode* makeRevoluteJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, 
  double width, double length, double z_len, double x_offset, double y_offset, double z_offset);
  void setGeometry(const SkeletonPtr& skel, const std::string& name, int degree, int dof);
  BodyNode* makeWeldJoint (const SkeletonPtr& skel, BodyNode* parent, const std::string& name, 
  double width, double length, double z_len, double x_offset, double y_offset, double z_offset);
};
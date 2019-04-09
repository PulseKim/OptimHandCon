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
  void makeBall(const SkeletonPtr& floor);
  void makeFinger(const SkeletonPtr& skel);
  void setGeometry(const SkeletonPtr& skel, const std::string& name, int degree, int dof);

  BodyNode* makeRoot(const SkeletonPtr& skel, const std::string& name);
  BodyNode* makeBallJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name);

  BodyNode* makeRevoluteJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name);


  ///Overload Joint makers

  BodyNode* makeBallJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, 
  double width, double length, double z_len, double x_offset, double y_offset, double z_offset, double limit_upper_x, double limit_lower_x, double limit_upper_y, double limit_lower_y, double limit_upper_z, double limit_lower_z);

  BodyNode* makeRevoluteJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, 
  double width, double length, double z_len, double x_offset, double y_offset, double z_offset, const Eigen::Vector3d axis, double limit_upper, double limit_lower);

  BodyNode* makeWeldJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, 
  double width, double length, double z_len, double x_offset, double y_offset, double z_offset);

  BodyNode* makeUniversalJoint(const SkeletonPtr& skel, BodyNode* parent, const std::string& name, 
  double width, double length, double z_len, double x_offset, double y_offset, double z_offset, const Eigen::Vector3d axis1, const Eigen::Vector3d axis2, double limit_upper_1, double limit_lower_1, double limit_upper_2, double limit_lower_2);

};
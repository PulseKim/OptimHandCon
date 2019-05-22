#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "HandMaker.hpp"
#include "Dynamics.hpp"

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::math;

const int default_countdown = 100;
const int default_countdown_movement = 200;
const double default_force = 500.0;



class MyWindow : public SimWindow
{
public:
  /// Constructor
  MyWindow(const WorldPtr& world);

  void initSkeleton();

  void initTendon();

  void initSkeletonFinger();

  void poseSetter();

  void initTendonFinger();

  void basicMovement();
  
  void showTorque();

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override;

  void timeStepping() override;

  void drawTendonSimple();

  void drawTendon();

  Eigen::Vector3d getMidPoint(Eigen::Vector3d first, Eigen::Vector3d second, Eigen::Vector3d first_dir, Eigen::Vector3d second_dir);

  void drawMultipleTendons();

  void drawTarget();

  void draw() override;

  void targetMovement();

  double radian(double angle);

  void setTarget();

  void setPretarget();

protected:
	int mForceCountDown;
  int mPoseCountDown;

  std::unique_ptr<Controller> mController;
  SkeletonPtr finger;
  SkeletonPtr floor;
  SkeletonPtr hand;
  SkeletonPtr ball;
  std::vector<Tendon*> mTendon;
  std::vector<std::pair<int, Tendon*>> mFingerTendon;
  Eigen::VectorXd defaultPose;
  Eigen::VectorXd goalPose;
  std::vector<std::pair<Eigen::Vector3d, int>> Ends;

};

#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "Controller.hpp"
#include "HandMaker.hpp"


using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::math;

const int default_countdown = 100;
const double default_force = 500.0;


class MyWindow : public SimWindow
{
public:
  /// Constructor
  MyWindow(const WorldPtr& world);

  void initSkeleton();

  void initTendon();

  void initSkeletonFinger();
  void initTendonFinger();

  void basicMovement();
  
  void showTorque();

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override;

  void timeStepping() override;

  void drawTendonSimple();

  void drawTendon();

  Eigen::Vector3d getMidPoint(Eigen::Vector3d first, Eigen::Vector3d second, Eigen::Vector3d first_dir, Eigen::Vector3d second_dir);


  void draw() override;

protected:
	int mForceCountDown;
	std::unique_ptr<Controller> mController;
	SkeletonPtr finger;
	SkeletonPtr floor;
	SkeletonPtr hand;
	std::vector<Tendon*> mTendon;
};

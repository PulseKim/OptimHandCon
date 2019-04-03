#include <iostream>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/utils.hpp>
#include "MyWindow.hpp"
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::glut;
using namespace dart::math;



int main(int argc,char** argv)
{
	WorldPtr world = std::make_shared<World>();
	MyWindow window(world);
	glutInit(&argc, argv);
	window.initWindow(1280,720, "Finger");
	glutMainLoop();

	return 0;
}
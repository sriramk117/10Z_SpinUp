#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor frontRight = motor(PORT13, ratio6_1, false);
motor frontLeft = motor(PORT12, ratio6_1, true);
motor backRight = motor(PORT17, ratio6_1, false);
motor backLeft = motor(PORT16, ratio6_1, true);
inertial Gyro = inertial(PORT11);
motor dr4b = motor(PORT3, ratio18_1, true);
controller Controller1 = controller(primary);
encoder LE = encoder(Brain.ThreeWirePort.C);
motor convey = motor(PORT19, ratio6_1, true);
motor fourBar2 = motor(PORT1, ratio18_1, true);
motor twoBar = motor(PORT10, ratio18_1, true);
motor tilter = motor(PORT4, ratio18_1, true);
distance DistanceBack = distance(PORT2);
rotation RotationTilter = rotation(PORT5, false);
distance DistanceFront = distance(PORT21);
/*vex-vision-config:begin*/
signature vision_sensor__GOAL_RED = signature (1, -1, 151, 75, -4553, -3821, -4187, 3, 0);
signature vision_sensor__GOAL_BLUE = signature (2, -2935, -2581, -2758, 8247, 8705, 8476, 3, 0);
signature vision_sensor__MOGORED = signature (3, 6431, 6843, 6637, 301, 527, 414, 2.5, 0);
vision vision_sensor = vision (PORT7, 50, vision_sensor__GOAL_RED, vision_sensor__GOAL_BLUE, vision_sensor__MOGORED);
/*vex-vision-config:end*/
motor middleLeft = motor(PORT14, ratio6_1, true);
motor middleRight = motor(PORT15, ratio6_1, false);
motor flywheel = motor(PORT18, ratio18_1, false);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.H);
optical Optical = optical(PORT9);
distance Distance = distance(PORT8);
bumper Bumper = bumper(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
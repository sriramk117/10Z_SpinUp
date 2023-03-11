using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern motor frontRight;
extern motor frontLeft;
extern motor backRight;
extern motor backLeft;
extern inertial Gyro;
extern motor dr4b;
extern controller Controller1;
extern encoder LE;
extern motor convey;
extern motor fourBar2;
extern motor twoBar;
extern motor tilter;
extern distance DistanceBack;
extern rotation RotationTilter;
extern distance DistanceFront;
extern signature vision_sensor__GOAL_BLUE;
extern signature vision_sensor__GOAL_RED;
extern signature vision_sensor__MOGORED;
extern signature vision_sensor__SIG_4;
extern signature vision_sensor__SIG_5;
extern signature vision_sensor__SIG_6;
extern signature vision_sensor__SIG_7;
extern vision vision_sensor;
extern motor middleLeft;
extern motor middleRight;
extern motor flywheel;
extern digital_out indexer;
extern digital_out DigitalOutH;
extern digital_out compression;
extern optical Optical;
extern distance Distance;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
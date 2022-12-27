#include <cmath>
#include <vector>


using namespace std;


class odom {
  public:
  

  /*TODO: Make all functions and variables private except active and positionTracking
          Make all functions and variables not static except active and positionTracking 
          Make constructors and destructors for the odom class
          Add the function TrackPosition to the header file
          Remove "odom::" from all the variables in TrackPosition method
    */
  //////////////////////////////INSTANCE VARIABLES\\\\\\\\\\\\\\\\\\\\\\\\\\
  
  static bool active;
  // Values measured by threaded function positionTracking
  static double x; //WILL HAVE TO CHANGE DATATYPE WHEN THREADING ODOMETRY TRACKING FUNCTION; Variable for current x and y coordinates
  static double y; //WILL HAVE TO CHANGE DATATYPE WHEN THREADING ODOMETRY TRACKING FUNCTION; Variable for current x and y coordinates
  static double a; //angle
  //vector<double> target_Coordinates = {target_x, target_y}; // Vector which store the target x and y coordinates for organization
  
  static void positionTracking();
  vector<float> time_List = {};
  vector<vector <double> > vector_List = {};
  //float global_time; Trial
  
  static void moveTo(double target_x, double target_y, bool f, bool head_x); //change parameters
  static void standardDrive(double lon, double rot);
  static void turnTo(double desiredHeading);
  static void moveForward(double lon);
  //static void moveForwardPID(double x, double y, bool f);
  static void moveForwardPID(double d, double cap);
  static void moveForwardOdom(double d, double v);
  static void moveReverse(double lon);
  static void moveChainBar(double pos);
  static void reverseTurn(double desiredHeading);


  // Member functions
   odom();

  private:
  // Values for positionTracking
  double TL; //Distance between left encoder and absolute robot position
  double TR; //Distance between right encoder and absolute robot position
  double TB; //Distance between back encoder and absolute robot position
  double angle; //Current rotation value of robot respective to field
  double theta0; //Previous Orientation
  double theta1; //Current Orientation
  double dTheta; //Change in Robot Angle
  double thetaM; //Average Orientation
  double dOffset; //Local Offset
  double dx; //Offset in x
  double dy; //Offset in y
  double DL; //Distance of wheel travel based on change in Left Encoder values
  double DR; //Distance of wheel travel based on change in Right Encoder values
  double DB; //Distance of wheel travel based on change in Back Encoder values
  double z; //Average of DR and DL

  // Wheel 
  double wheelradius; 
  double wheelCircumference;
  //static double offsetBack;
  //static double offsetMiddle; 
  
  /*
  // Variables for moveTo
  static double target_x; // Variable for x value of target coordinate
  static double target_y; // Variable for y value of target coordinate
  static vector<double> prev_Coordinates = {prev_x, prev_y}; // Vector which stores the current x and y coordinates for organization
  static double error_x = target_x-prev_x; // Value which must be added to current x position to get target x position (Used for Calculating Header For Gyro) 
  static double aerror_x = abs(error_x); // Absolute Value Calculated for Finding the side lengths of right triangle for calculate theta value
  static double error_y = target_y-prev_y; // Value which must be added to current x position to get target y position (Used for Calculating Header For Gyro) 
  static double aerror_y = abs(error_y); // Absolute Value Calculated for Finding the side lengths of right triangle for calculating theta value
  static vector<double> changeVector = {error_x, error_y}; // Vector which stores a "vector" from current coordinates to target coordinates including change in x and y 
  static vector<double> triangle = {aerror_x, aerror_y}; // Vector which stores the length for the two legs of the right triangle used for calculating theta
  static double current_heading; // The current heading of the gyro sensor
  static double target_heading; // THe target heading of the gyro sensor
  static double theta = atan(aerror_y/aerror_x); // The value of the smaller angle between the hypotenus and the line y=(current x) of the right triangle between the current coordinates and the target coordinates
  static double converted_Theta;
  static vector<double> vector_Tracking = {};
  */



  //////////////////////////////FUNCTIONS\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  // Testing
  void testCalibration();
  
  // Math Functions
  double radianConversion(double theta);

  // Position Tracking
  void TrackPosition(); //change parameters

  // Odometry Movement
  double timeTracker(float a, float b);
  double vectorTracker();

  


  


  

};

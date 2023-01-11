#include <iostream>

using namespace std;
typedef long long ll;
typedef int i;
typedef double d;
typedef string s;


class pid {
  public:
  // Constructor
  pid();
  
  //Variables


  // Methods For Motion Algorithms
  double gyroPid(double angle);
  double strafe(double dx);
  double axial(double dy, double pidCap);
  void reinitialize();

  //Methods For Life Mechanisms
  void dr4b(double height);


  //PID Methods
  //static double gyroPid(double angle);
  //static double strafe(double dx);
  //static double axial(double dy);

  private:

  //Settings for gyroPID 
  double turn_PidFloor;
  double turn_PidCap;
  double turn_kD;
  double turn_kP;
  double turn_kI;
  double turn_prevError;
  

  //Settings for axialPID 
  double axial_PidFloor;
  double axial_PidCap;
  double axial_kD;
  double axial_kP;
  double axial_kI; 
  double axial_prevError;  



};
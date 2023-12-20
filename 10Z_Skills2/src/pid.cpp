#include "vex.h"
#include "pid.h"
#include "odom.h"

using namespace std;

double t_error = 0;
double t_integral = 0;
double t_derivative = 0;

double a_error = 0; //Sensor Value - Desired Value : Position Value
double a_derivative = 0; // Derivative -> Gives Speed
double a_integral = 0; // Integral -> Gives Postion

pid::pid() {
  axial_prevError = 0;
  turn_prevError = 0;
}

double pid::axial(double dy, double pidCap) {
  //Settings for PID
  axial_kP = 0.69;
  axial_kI = 0;
  axial_kD = 0.63;
  axial_PidCap = pidCap;
  
  double vel = 5; // Velocity
  
  ///////////////////////////////////P - I - D 
  
  //Error
  a_error = dy;
  //Derivative
  a_derivative = a_error - axial_prevError;
  //Integral
  a_integral += a_error;

  
  
  a_integral  = a_integral + a_error;
  if(a_error == 0 || a_error < dy) {
    a_integral = 0;
  }

  axial_prevError = a_error;
  vel = (a_error*axial_kP + a_derivative * axial_kD + a_integral * axial_kI);

  if (abs(vel) < 100) {
    if (vel < 0) {
      vel = -100; 
    } else {
      vel = 100; 
    }
  }
  
  if(abs(vel) > axial_PidCap) {
    if (vel < 0) {
      vel = -axial_PidCap;
    } else {
      vel = axial_PidCap;
    }
  }
  
  return vel;
}

double pid::gyroPid(double phi, double pidCap) {
  double vel = 0;
  double threshold;
  if(phi <= 0.0) {
    threshold = 1.5; 
  } else {
    threshold = 0.1; 
  }

  turn_kP = 3.0;
  turn_kD = 2.73;
  turn_kI = 0;
  turn_PidCap = pidCap;
  
  t_error = phi;
   
  bool forward = (phi >= Gyro.heading(degrees));
   
  t_integral  = t_integral + t_error;
  if(t_error == 0 || ((t_error < phi) && forward) || ((t_error >= threshold) && !forward)) {
    t_integral = 0;
  }
    
  t_derivative = t_error - turn_prevError;
  turn_prevError = t_error;

  double p = t_error * turn_kP;
  double i = t_integral * turn_kI;
  double d = t_derivative * turn_kD;
  vel = p + i + d;
     
  if(abs(vel) > turn_PidCap) {
    if (vel < 0) {
      vel = -turn_PidCap;
    } else {
      vel = turn_PidCap;
    }
  }

  return vel;
}

void pid::reinitialize() {
  t_error = 0;
  t_integral = 0;
  t_derivative = 0;

  a_error = 0; 
  a_derivative = 0; 
  a_integral = 0;

  axial_prevError = 0;
  turn_prevError = 0;
}

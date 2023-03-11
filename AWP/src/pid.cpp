#include "vex.h"
#include "pid.h"
#include "odom.h"

using namespace std;

double t_error = 0;
double t_integral = 0;
double t_derivative = 0;

double a_error = 0; //SensorValue-Desired Value : Position Value
//double prevError = 0; // Position 20 miliseconds ago                          ALREADY INITIALIZED IN /pid.h
double a_derivative = 0; // Derivative -> Gives Speed
double a_integral = 0; // Integral -> Gives Postion

pid::pid() {
  axial_prevError = 0;
  turn_prevError = 0;
}

double pid::axial(double dy, double pidCap) {
  //double degree = inches / 0.0385;
  //Settings for PID
  axial_kP = 0.69;//0.55;//0.3;//0.55;//0.3;//.5;
  axial_kI = 0;//0.001;//0.000001;//0.000001;//0.05;
  axial_kD = 0.63;//0.65;//0.8;//0.1;//0.06;//0.52;
  axial_PidCap = pidCap;//1000000000000;
  //Measured Values
  //double desiredValue = degree;

  
  double vel = 5; // Velocity
  
  //LE.setPosition(0, degrees);
  //RE.setPosition(0, degrees);
  
  //double robotPosition = (fabs(LE.position(degrees)) + fabs(RE.position(degrees)))/2;

  //Brain.Screen.print("error: %f", error);
  //Brain.Screen.newLine();

  //int FrontLeftMotorPosition = FrontLeftMotor.position(degrees);
  //int BackRightMotorPosition = BackRightMotor.position(degrees);
  //int BackLeftMotorPosition = BackLeftMotor.position(degrees);
  
  //AveragePosition
  //int averagePositionForMotor = (-FrontRightMotorPosition + FrontLeftMotorPosition - BackRightMotorPosition + BackLeftMotorPosition)/4;
  
  ///////////////////////////////////P - I - D 
  //Error
  a_error = dy;
  //Derivative
  a_derivative = a_error - axial_prevError;
  //Integral
  a_integral += a_error;
  //Speed
  //while(error >= 0){
  
  
  a_integral  = a_integral + a_error;
  if(a_error == 0 || a_error < dy) {
    a_integral = 0;
  }

  axial_prevError = a_error;
  vel = (a_error*axial_kP + a_derivative * axial_kD + a_integral * axial_kI);
  //Brain.Screen.print("error:%f", a_error);

  //if (vel > axial_PidCap) {
    //vel = axial_PidCap;
  //} 
  //cout << "vel: "<< vel << "\n" ;
  
  /*if (abs(vel) < 20){
    if (vel < 0) {
      vel = -20;
    } else {
      vel = 20;
    }
  }*/

  if (abs(vel) < 100) {
    if (vel < 0) {
      vel = -100; //-100
    } else {
      vel = 100; //100
    }
  }
  
  if(abs(vel) > axial_PidCap) {
    if (vel < 0) {
      vel = -axial_PidCap;
    } else {
      vel = axial_PidCap;
    }
  }
  //cout << "Vel: " << vel << "\n";

  //cout << "vel: "<< vel << "\n" ;

  
  /*  
  frontRight.spin(fwd, vel, rpm);
  backRight.spin(fwd, vel, rpm);
  frontLeft.spin(fwd, vel, rpm);
  backLeft.spin(fwd, vel, rpm);*/
  wait(1, msec);
  ////////////////////////////////
  //robotPosition = (fabs(LE.position(degrees)) + fabs(RE.position(degrees)))/2;
  //error = desiredValue - robotPosition; //robot position should be equal to y value returned by position tracking algorithm
  //Brain.Screen.print("vel: %f", vel);
  //Brain.Screen.newLine();
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
 
  // variable instantiations
  turn_kP = 3;//2.8;//2.75;//2.65;//2.6;//3.5;//3.1;//3.4;//2.4;//2;//0.94;//2;//0.85;//0.9;
  turn_kD = 2.8;//2.73;//2.73;//2.8;//2.71;//2.9; 
  turn_kI = 0.000000001;   
  turn_PidCap = pidCap;
  //double vel = 0;
  //int badnums = 0;
  t_error = phi;
   
   
  bool forward = (phi >= Gyro.heading(degrees));//(phi >= odom::a);
   
   
  t_integral  = t_integral + t_error;//t_integral + fabs(t_error);
  if(t_error == 0 || ((t_error < phi) && forward) || ((t_error >= threshold) && !forward)) {
    t_integral = 0;
  }
    
  t_derivative = t_error - turn_prevError;//fabs(t_error) - fabs(turn_prevError);
  turn_prevError = t_error;

  //Brain.Screen.print("error:%f", t_error);

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

  /*
  if (abs(vel) < 25) {
    if (vel < 0) {
      vel = -25; //-100
    } else {
      vel = 25; //100
    }
  }*/

  return vel;

  //wait(1, msec);


    
  /////////////////////////////////
  

  //Brain.Screen.print("END hd = %f, err=%f, vel=%f", Gyro.heading(degrees), error, vel);
  //Brain.Screen.newLine();

  //if(Brain.Screen.row() > 10) {
    //Brain.Screen.setCursor(0,1);
  //}
  //if ((forward &&(odom::a > phi)) || (!forward && (phi > odom::a)) ) badnums++;
  /*
  Brain.Screen.print("========================================");
  Brain.Screen.newLine();
  Brain.Screen.print("STOP bads = %d,  hd = %f, err=%f, vel=%f",badnums, odom::a, error, vel);
  Brain.Screen.newLine();*/
  //cout << "vel: "<< vel << "\n" ;
  //if (!forward) threshold = -0.1; 
     // t_error = phi;
    //while (((error > threshold) && forward) || ((error < threshold) && !forward)) {                                                                  
    
  //Brain.Screen.print("BEG hd = %f, err=%f", odom::a, error);
  //Brain.Screen.newLine();

  //if (vel < 10) vel = 10;
  /*
  if (abs(vel) < 10){
    if (vel < 0) {
      vel = -10;
    } else {
      vel = 10;
    }
  }*/
  //Brain.Screen.print("velocity = %f", vel);
  //Brain.Screen.newLine();
    
  
  
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
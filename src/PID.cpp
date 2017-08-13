#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() :Kp(0.0),Ki(0.0),Kd(0.0),p_error(0.0),i_error(0.0),d_error(0.0),total_error(0.0){}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp=Kp;
  this->Ki=Ki;
  this->Kd=Kd;
  p_error=0.0;
  i_error=0.0;
  d_error=0.0;
  total_error=0.0;
}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  p_error = cte;

  //Prevent wind up. Integral term should not be commanding more than max steering angle
  if(fabs(Ki*(i_error+cte))<1.0){
    i_error+=cte;
  }

  total_error+=fabs(cte);

}

double PID::GenerateCommand(){
  return -(Kp*p_error + Ki*i_error + Kd*d_error);
}

double PID::TotalError() {
  double return_val = total_error;
  total_error = 0.0;
  return return_val;
}


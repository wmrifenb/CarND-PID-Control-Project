#include "PID.h"
#include <math.h>
#include <cstdio>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() :p({1,0,0}),dp({0.01, -0.001, 0.01}),p_error(0.0),i_error(0.0),d_error(0.0),total_error(0.0),best_error(9E10),ts_(stage_one),gs_(proportional){}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p={Kp, Ki, Kd};
  dp={0.01, -0.0001, 0.01};
  p_error=0.0;
  i_error=0.0;
  d_error=0.0;
  total_error=0.0;
  best_error=9E10;
  ts_=stage_one;
  gs_=proportional;
}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  p_error = cte;

  //Prevent wind up. Integral term should not be commanding more than max steering angle
  if(fabs(p[1]*(i_error+cte))<1.0){
    i_error+=cte;
  }

  total_error+=fabs(cte);

}

double PID::GenerateCommand(){
  return -(p[0]*p_error + p[1]*i_error + p[2]*d_error);
}

void PID::moveToNextGain(){
  if(gs_==proportional){gs_=integral; return;}
  if(gs_==integral){gs_=derivative; return;}
  if(gs_==derivative){gs_=proportional; return;}
}

void PID::Twiddle(){
  if( (dp[0] + dp[1] + dp[2]) > 0.0001 ){

    if(ts_==stage_one){
      if(total_error < best_error){
        best_error = total_error;
        dp[gs_] *= 1.1;
        moveToNextGain();
        p[gs_] += dp[gs_];
        total_error = 0.0;
        return;
      }else{
        p[gs_] -= 2 * dp[gs_];
        ts_=stage_two;
        total_error=0.0;
        return;
      }
    }
    if(ts_==stage_two){
      if(total_error < best_error){
        best_error = total_error;
        dp[gs_] *= 1.1;
        moveToNextGain();
        p[gs_] += dp[gs_];
        ts_=stage_one;
        total_error=0.0;
        return;
      }else{
        p[gs_] += dp[gs_];
        dp[gs_] *= 0.9;
        moveToNextGain();
        p[gs_] += dp[gs_];
        ts_=stage_one;
        total_error=0.0;
        return;
      }
    }
  }else{
    cout << "Twiddle converged! P: " << p[0] << ", I: " << p[1] << ", D: "<< p[2] << endl;
    return;
  }
}


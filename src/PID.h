#ifndef PID_H
#include <vector>
#define PID_H

enum twiddle_stage {stage_one, stage_two};
enum gain_stage {proportional=0, integral=1, derivative=2};

class PID {
public:

  //Coefficients
  std::vector<double> p;
  std::vector<double> dp;

  //Errors
  double p_error;
  double i_error;
  double d_error;
  double total_error;
  double best_error;

  //Constructor
  PID();

  //Destructor
  virtual ~PID();

  //Initialize PID.
  void Init(double Kp, double Ki, double Kd);

  //Update the PID error variables given cross track error.
  void UpdateError(double cte);

  //Use twiddle algorithm to update gains
  void Twiddle();

  void moveToNextGain();

  //Generate command signal.
  double GenerateCommand();

private:
  twiddle_stage ts_;
  gain_stage gs_;

};

#endif /* PID_H */

# PID Control Project

This project displays tunning of a PID controller for a simulation of an autonomous driving car using twiddle. The simulation maybe found here: [Udacity Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)

The twiddle tunning program makes use of a proportional controller that governs the throttle of the vehicle to make it go a constant speed. 20 mph was chosen.

While the vehicle laps around the course a timer keeps track of duration on the course. Every time roughly 2 minutes and 20 seconds pass (The time it takes to complete a lap at 20mph), the twiddle function is called and new gains are chosen for a trial lap. The total error for each trial lap is measured and then used to determine if the adjustment in the gains has improved performance since the previous set of gains as in the classic twiddle method. Below is the code for the twiddle method as found in PID.cpp:

```C++
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

```

The gains ultimately converged to P=1.011, I=-0.005, D=0.01 but never trip off the sumation convergence criterion.
This is most likely due to the method of using time to keep track of trials rather than a specific marker on the course of some sort, leading to less than perfect comparisons of total trial error.

Below is a link to a video showing the PID gains in action. The speed was set to 15mph though because the simulation would appear to have latency issues when video recording software was activated. The PID controller would perform more poorly than without video recording.

[15mph PID Steering controller](https://youtu.be/TGOFMxPtl_E)





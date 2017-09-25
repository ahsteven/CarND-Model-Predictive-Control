
## Udacity Self-Driving Car Nanodegree MPC Project Discussion

Please see youtube video of working model:

### Video of Second Order Polyfit of Railing
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/9hKtK_u5w7E/0.jpg)](https://www.youtube.com/watch?v=9hKtK_u5w7E)

#### Udacity Simulator
This project controoled a vehicle in a Unity Simulator provided by udacity here:  https://github.com/udacity/self-driving-car-sim/releases

The C++ code uses the IPOPT and CPPAD libraries to run the optimization each time step.

The c++ code communicates with the Simulator through UWebSockets: https://github.com/uNetworking/uWebSockets

cmak


#### Model Discription:

The state of the system is described by: x location, y location, heading angle, velocity, cross track error (cte), and heading error (epsi). 

The state equations are used as constraints for the solver. The state equazions are below:
```cpp
  // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
  // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
  // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
  // v_[t] = v[t-1] + a[t-1] * dt
  // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
  // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt  
```
The new x and y positions are determined by the old plus the cos(heading) times the expected distance traveled as estimated by the current velocity.

The new heading angle is determined by the old plus the shift caused by the steering angle, and the distance traveled as well as the vehicle length. Here the steering angle is an input actuation command determined by the solver.

The new velocity is determined by the old velocity plus the acceleration times dt. Here acceleration is an input actuation command that is determined by the solver. 

The cross track error is determined by expected polynomial value at time t minus the expected position of the car at time t.

The heading error is determined by the current heading minus the desired heading. The desired heading is determeined by taking the tangent of the polynomial of the desired waypoint path and caclulating the angle of that tangent with respect to the current robot heading (=0 robot reference frame).


So in the solver the next time step must equal the current time step plus the prected change. The solver solves them by reducing their difference to zero as shown below:
```cpp
// polynomial y value at x0 
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      // desired heading angle
      // determined by taking angle of tangent of polynomial fit of desired waypoint path
      AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);


  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);// - delta
  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
  fg[1 + cte_start + t] =
    cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
  fg[1 + epsi_start + t] =
    epsi1 - (psi0 - psides0 + v0 * delta0 / Lf * dt);
```

The actuator s are variables that are optimized by the cost function:
For the cost function I chose to minimize cte, hading error, desired velocity - actual velocity. Also, I decided to reduce the use of the acceleration pedal to reduce fuel consumption. There is no loss in using the steering actuation and sometimes you just need to turn so it was not included as a cost. The change in steering however was included in the cost function so that the ride is smoother. Also the change in acceleration is included. 

```cpp
    for (int t = 0; t < N; t++) {
      fg[0] += 3*CppAD::pow(vars[cte_start + t], 2);// cross track error 
      fg[0] += 750*CppAD::pow(vars[epsi_start + t], 2); // heading angle and desired heading angle 6000
      //fg[0] += 2*CppAD::pow(vars[v_start + t] - ref_v, 2); // velocity follow reference
      fg[0] += 2*CppAD::pow(vars[v_start + t] - ref_v2, 2);
    }
    //Minimize the use of accelerator.
    for (int t = 0; t < N - 1; t++) {
        fg[0] += CppAD::pow(vars[a_start + t], 2);// minimize |acceleration|
    }
    //Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      //reduce change in steering and acceleration
       fg[0] += 20000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);//200
       fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }  
```

#### Controlling Speed Reference

The speed reference was determined by the predicted steering angle in 500 ms. A second order polynomial fit equation was calculated that had a max speed equal to 100 mpy when the steering angle was zero. Then as the steering angle increased the velocity reference decreased. So with a steering angle of .1 radians the speed should decrease to 38 mph.

The future steering value is set by:
```cpp
   // future steering angle to determine speeed
  future_steer = -solution.x[delta_start+5];
```

The speed reference is then determined by:
```cpp
double ref_v = 100*.447;// 100 mph to m/s

double ref_v2 =424*pow(future_steer,2) - 
    272*fabs(future_steer) + ref_v;  
```
#### Timestep Length:

A timestep of .1 was chosen because it gave an exceptable response to commands and the computation time was resonable. First then number of steps was kept at 10. Then it was increased to 15 to see further out. However as the complexity of the model grew with the speed control optimization the number of timesteps was set to 13 which provided similar results but less delay. The solver itself could in some of the worst cases take .05 seconds to optimize the variables setting the time step this low was problematic.

#### Polynomial Fitting:
A set of waypoints is read in from the simulator in world coordinates. These waypoints are then converted to the robot reference frame first by a translation and then by a rotation.
The udacity simulator has angles going clockwise as opposed to the normal mathematically accepted counterclockwise. So the - of the heading angle of the vehicle is used in the rotation matrix. THe code is shown below:

```cpp
  for (int i = 0; i < ptsx.size(); i++)
  {

      double shift_x = ptsx[i] - px;
      double shift_y = ptsy[i] - py;
      ptsx[i] = shift_x*cos( -psi) - shift_y*sin( -psi);
      ptsy[i] = shift_x*sin( -psi) + shift_y*cos( -psi);

  }
```
These points are then interpolated to 6 points which are then fed into the polyfit function. The polyfit is set to return a 3rd order polynomial.

The cross track error is then determined by evaluating the polynomial at x = 0; This gives the distance to the point on the polynomial that is to the direct left or direct right of the car. 

The error in heading angle is then determined by getting the tangent of the polynomial at x = 0. To find the angle between our heading and the tangent of the polynomial the negative arctan of the angle is used:
```cpp
 // angle between current heading and slope of the poly fit function at x = 0 
 double epsi = -atan(coeffs[1]);
```
This is the current cte and epsi state values that initialize the solver. 

#### Handeling Latency:
A latency was added after the control commands were determined to simulate the amount of time it would take for the actuation to occur after the value was determined. Also there is a latency caused by the solver that was measured to be around .3 seconds. To deal with this the steering command  was set to an average of the optimal predicted steering command at 100 ms and the optimal predicted steering command at 200 ms. This helped mitigate the latency issues.
```cpp
  // take average of predected angle between 100 ms and 200 ms ahead
  steer_val =  solution.x[delta_start+1]*.5+solution.x[delta_start+2]*.5;
```

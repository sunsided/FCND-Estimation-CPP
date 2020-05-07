# Quadrotor Sensor Fusion and Pose Estimation

In this continuation of the [controls project](https://github.com/sunsided/FCND-Controls-CPP), 
the sensor fusion and pose estimation code for a your simulated quad drone
will be implemented. 

💡 **Fun fact:** Did you know that the word 🚁 "helicopter" isn't composed of the parts `heli` and `copter`, but of the parts `helico` and `pter`, from greek _helix_ (spiral) and _pteron_ (winged; think [Pterodactylus](https://en.wikipedia.org/wiki/Pterodactylus))? So the pronounciation really should be helico-pter, and a "quadrocopter" actually is a "quadropter". Thanks for listening, now go have fun. 😁

## Project Structure

Here are a couple of interesting points for working with the repo:

 - An Extended Kalman Filter is implemented in `QuadEstimatorEKF.cpp`
 - Parameters for tuning the EKF are in the parameter file `QuadEstimatorEKF.txt`
 - When various sensors are turned on (the scenarios configure them, e.g. 
   `Quad.Sensors += SimIMU, SimMag, SimGPS`), additional sensor plots will become available to see what
   the simulated sensors measure.
 - The EKF implementation exposes both the estimated state and a number of additional variables.
   In particular:
   - `Quad.Est.E.X` is the error in estimated X position from true value.  More generally, the variables in `<vehicle>.Est.E.*` are relative errors, though some are combined errors (e.g. MaxEuler).
   - `Quad.Est.S.X` is the estimated standard deviation of the X state (that is, the square root of the appropriate diagonal variable in the covariance matrix). More generally, the variables in `<vehicle>.Est.S.*` are standard deviations calculated from the estimator state covariance matrix.
   - `Quad.Est.D` contains miscellaneous additional debug variables useful in diagnosing the filter. You may or might not find these useful but they were helpful to us in verifying the filter and may give you some ideas if you hit a block.

### `config` Directory

In the `config` directory, in addition to finding the configuration files for your controller and your
estimator, you will also see configuration files for each of the simulations.  For this project, you
will be working with simulations 06 through 11 and you may find it insightful to take a look at the
configuration for the simulation.

As an example, if we look through the configuration file for scenario 07, we see the following parameters
controlling the sensor:

```
# Sensors
Quad.Sensors = SimIMU
# use a perfect IMU
SimIMU.AccelStd = 0,0,0
SimIMU.GyroStd = 0,0,0
```

This configuration tells us that the simulator is only using an IMU and the sensor data will have no noise.
You will notice that for each simulator these parameters will change slightly as additional sensors are
being used and the noise behavior of the sensors change.

## The Steps

For a description of the original project tasks, see [TASKS.md](TASKS.md).

### Evaluating Sensor Noise

For the [controls project](https://github.com/sunsided/FCND-Controls-CPP), the simulator was working
with a set of entirely noise-free sensors. In order to add realism to the problem, noise is now added
back in. To do so, some noisy sensor data is collected in simulator scenario `06_NoisySensors`
and then evaluated.

In this scenario, a drone is kept motionless while sensor data is collected.
When run, two CSV files are created:

- [`config/log/Graph1.txt`](config/log/Graph1.txt) contains GPS X position measurements, and
- [`config/log/Graph2.txt`](config/log/Graph2.txt) contains accelerometer data for the X direction.

The Python script [`scripts/determine-sensor-noise.py`](scripts/determine-sensor-noise.py) was written
to evaluate the data. When executed like so,

```bash
scripts/determine-sensor-noise.py config/log/Graph1.txt
```

it gives something along the lines of

```
Found field names:
- Quad.GPS.X

Statistics:
- Series:     Quad.GPS.X
  Count:      23
  Min:        -1.571771
  Max:        0.976935
  Mean:       -0.06606665217391307
  Std. Error: 0.1379983888851819
  Std. Dev:   0.661817023581923
```

Note that `time` data has been removed from the output for brevity. A Conda environment is available
in [`environment.yaml`](environment.yaml) and can be set up using

```bash
conda env create -f environment.yaml
conda activate udacity-fcnd
```

The obtained standard deviations are

- **GPS X/Y:** `0.7077498542201446`
- **Accelerometer X/Y:** `0.4891275427009619`

The [`config/06_SensorNoise.txt`](config/06_SensorNoise.txt) configuration file was updated with
the obtained standard deviations, after which the simulator correctly detects that approximately 68% of
all sensor data fall into the µ ± σ range. 
See [`config/SimulatedSensors.txt`](config/SimulatedSensors.txt) for sensor parameters shipped with
the starter code.


### Attitude Estimation by Complementary Filter

In theory, gyro data could be integrated constantly in order to obtain an attitude
estimate. Due to the noise characteristics of a gyro sensor however and the fact
that it gets integrated with every update, this approach performing extremely badly and
results in noticeable drift.

To mitigate, a complementary filter-type solution can be used that combines
attitude from integrated gyro roll rates with direct attitude estimation from the
accelerometer, thus combining fast updates from the gyro with long-term stability
of the accelerometer. 

In order to first perform attitude estimation from the gyro, the starter code used a
simple integration scheme like so: 

```c++
auto predictedPitch = pitchEst + dtIMU * gyro.y;
auto predictedRoll  = rollEst  + dtIMU * gyro.x;
auto predictedYaw   = yawEst   + dtIMU * gyro.z;
```

The main problem with the this approach is that it is only a
[small-angle approximation](https://en.wikipedia.org/wiki/Small-angle_approximation),
and - concretely - valid only with very small roll rates. Specifically is it based 
on the idea that a sinusoidal function is approximately linear in any small
environment around zero and follow directly from a Taylor series
approximation of the `sin` and `cos` functions (at zero).

![](images/small-angle-approximation.png)

To fix this, a quaternion-based integration was implemented to apply body frame
roll rates to inertial frame euler angles:

```c++
const auto orientation = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, yawEst);
const auto rollRate    = Quaternion<float>::FromAxisAngle(gyro * dtIMU);
const auto predictedOrientation = rollRate * orientation;

const auto predictedPitch = predictedOrientation.Pitch();
const auto predictedRoll  = predictedOrientation.Roll();
const auto predictedYaw   = wrapAngle(predictedOrientation.Yaw()); // -π .. +π
```

Here, `wrapAngle(radians)` is a helper function that ensures a provided angle is in -π .. +π range.

Lastly, the complementary filter fuses the gyro-predicted roll and pitch angle
with the accelerometer-predicted roll and pitch angles in the usual fashion:

```c++
const auto gyroWeight  = attitudeTau / (attitudeTau + dtIMU);
const auto accelWeight = 1 - gyroCoeff; // dtIMU / (attitudeTau + dtIMU);

rollEst  = wrapAngle(gyroWeight * predictedRoll  + accelWeight * accelRoll);
pitchEst = wrapAngle(gyroWeight * predictedPitch + accelWeight * accelPitch);
```

Here's the output after implementing the above:

![](images/scenario-2.png)

Note that the pitch and roll errors temporarily reach (only) about ±0.02 rad,
or about ±1.15°, and quickly go back to zero.

The yaw error permanently increases at the and of the cycle since the drone
is performing a rotation around its up axis. However, the yaw angle is not yet
fused with another sensor (note that in the above, yaw is only integrated from
roll rates, which leads to the aforementioned drift). To fix this, extra
information such as the magnetometer's absolute orientation information
will be required.

For comparison, here's an excerpt from the starter code:

> In the screenshot \[below] the attitude estimation using linear scheme (left) and using the improved nonlinear
  scheme (right) \[is shown]. Note that Y axis on error is much greater on left.
>
> ![attitude example](images/attitude-screenshot.png)


### EKF Prediction

The state and covariance prediction steps of the Extended Kalman Filter were
implemented using scenario `08_PredictState` and `09_PredictionCov` of the simulator.

Note that scenario `08_PredictState` uses only uses a noise-free IMU and suppresses
the accelerometer by using an extremely high complementary filter weight for the
gyroscope (`QuadEstimatorEKF.attitudeTau = 100`) in order to suppress
drift from the doubly integrated accelerometer.

Not much is to say here; the `PredictState()` method was implemented to fast-forward
the state vector:

```c++
const auto accelInertialFrame = bodyToInertialFrame(accel) - gravity;

predictedState(0) += predictedState(3) * dt;
predictedState(1) += predictedState(4) * dt;
predictedState(2) += predictedState(5) * dt;

predictedState(3) += accelInertialFrame.x * dt;
predictedState(4) += accelInertialFrame.y * dt;
predictedState(5) += accelInertialFrame.z * dt;
```

Here, the accelerometer data was used as control input `u` since it is a somewhat
reasonable approximation of the actual (human-provided) controls.

Then, the `Predict()` method was updated to fast-forward the
covariance matrix and produce a new state and covariance estimate:

```c++
// Build the transition function's Jacobian
gPrime(0,3) = dt;
gPrime(1,4) = dt;
gPrime(2,5) = dt;
gPrime(3,6) = (RbgPrime(0,0) * accel.x + RbgPrime(0,1) * accel.y + RbgPrime(0,2) * accel.z) * dt;
gPrime(4,6) = (RbgPrime(1,0) * accel.x + RbgPrime(1,1) * accel.y + RbgPrime(1,2) * accel.z) * dt;
gPrime(5,6) = (RbgPrime(2,0) * accel.x + RbgPrime(2,1) * accel.y + RbgPrime(2,2) * accel.z) * dt;

// Covariance update
ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```

Here, `RbgPrime` is determined by the `GetRbgPrime()` helper method, which in turn constructs
the Jacobian of the body-to-inertial frame rotation matrix (with the "g" in "Rbg" standing
for global):

```c++
// Roll
RbgPrime(0, 0) = -cosTheta * sinPsi;
RbgPrime(0, 1) = -sinPhi   * sinTheta * sinPsi - cosPhi * cosPsi;
RbgPrime(0, 2) = -cosPhi   * sinTheta * sinPsi + sinPhi * cosPsi;

// Pitch
RbgPrime(1, 0) =  cosTheta * cosPsi;
RbgPrime(1, 1) =  sinPhi   * sinTheta * cosPsi - cosPhi * sinPsi;
RbgPrime(1, 2) =  cosPhi   * sinTheta * cosPsi + sinPhi * sinPsi;
```

Lastly, [`config/QuadEstimatorEKF.txt`](config/QuadEstimatorEKF.txt) was updated
to provide X/Y position and velocity standard deviations (`QPosXYStd` and `QVelXYStd`
respectively) in order to capture the error ranges somewhat accurately over
a small time horizon of one second. This process was done entirely empirically
by eyeballing the error plots and resulted in the following values:

- **`QPosXYStd`:** `.05`
- **`QVelXYStd`:** `.2`

Here's about how it looks: 

![good covariance](images/predict-good-cov.png)

Note that while this is a picture from the starter code, it _somewhat_ resembles
the actual output.

### Step 4: Magnetometer Update

Up until now we've only used the accelerometer and gyro for our state estimation.  In this step, you
will be adding the information from the magnetometer to improve your filter's performance in estimating
the vehicle's heading.

1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn't
   been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real
   value (and the estimated standard deviation is also increasing).  Note that in this case the plot is
   showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the
   simulation runs.  You should also see the estimated standard deviation of that state (white boundary)
   is also increasing.
2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately
   captures the magnitude of the drift, as demonstrated in the figure below.
3. Implement magnetometer update in the function `UpdateFromMag()`.  Once completed, you should see a
   resulting plot similar to this one:

The drift magnitude:

![mag drift](images/mag-drift.png)

A good solution looks like this:

![mag good](images/mag-good-solution.png)

***Success criteria:*** *Your goal is to both have an estimated standard deviation that accurately
captures the error and maintain an error of less than 0.1rad in heading for at least 10 seconds of the
simulation.*

**Hint: after implementing the magnetometer update, you may have to once again tune the parameter
`QYawStd` to better balance between the long term drift and short-time noise from the magnetometer.**

**Hint: see section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj)
for a refresher on the magnetometer update.**


### Step 5: Closed Loop + GPS Update

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and
   ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right).
   As you see they are drifting away, since GPS update is not yet implemented.
2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in
   `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an
   ideal IMU.
3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
    ```
    #SimIMU.AccelStd = 0,0,0
    #SimIMU.GyroStd = 0,0,0
    ```
4. Tune the process noise model in `QuadEstimatorEKF.txt` to try to approximately capture the error you
   see with the estimated uncertainty (standard deviation) of the filter.
5. Implement the EKF GPS Update in the function `UpdateFromGPS()`.
6. Now once again re-run the simulation.  Your objective is to complete the entire simulation cycle with
   estimated position error of < 1m (you’ll see a green box over the bottom graph if you succeed).
   You may want to try experimenting with the GPS update parameters to try and get better performance.

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated
position error of < 1m.*

**Hint: see section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj)
for a refresher on the GPS update.**

At this point, congratulations on having a working estimator!

### Step 6: Adding Your Controller

Up to this point, we have been working with a controller that has been relaxed to work with an estimated
state instead of a real state.  So now, you will see how well your controller performs and de-tune your
controller accordingly.

1. Replace `QuadController.cpp` with the controller you wrote in the last project.
2. Replace `QuadControlParams.txt` with the control parameters you came up with in the last project.
3. Run scenario `11_GPSUpdate`. If your controller crashes immediately do not panic. Flying from an
   estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need
   to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning
   being effective) to stabilize it.  Your goal is to once again complete the entire simulation cycle
   with an estimated position error of < 1m.

**Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors
and de-tuning under those conditions first.**

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated
position error of < 1m.*


## Tips and Tricks

 - When it comes to transposing matrices, `.transposeInPlace()` is the function you want to use to
   transpose a matrix
 - The [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) document contains a
   helpful mathematical breakdown of the core elements on your estimator

## Submission

For this project, you will need to submit:

 - a completed estimator that meets the performance criteria for each of the steps by submitting:
   - `QuadEstimatorEKF.cpp`
   - `config/QuadEstimatorEKF.txt`

 - a re-tuned controller that, in conjunction with your tuned estimator, is capable of meeting the criteria laid out in Step 6 by submitting:
   - `QuadController.cpp`
   - `config/QuadControlParams.txt`

 - a write up addressing all the points of the rubric

## Authors

Thanks to Fotokite for the initial development of the project code and simulator.

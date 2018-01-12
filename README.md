# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## PID gain optimization

To improve the performance of steering controller I implemented twiddle to tune the gains. Twiddle works as follow.

1. I assigned default gains all of proportional, integral and derivative controllers, and run simulation using default gains, and record current error.
2. Then I increment proportional gain by the delta rate of 0.1, and run simulation to see if the new error beats the previous best error.
3. If the new error beats the previous best, then I multiply the delta rate of increasing by 1.1, and see if simulation gives better (lower) error.
4. When the new error from incrementing does not beat the previous best, I decrease the gain by 2 times of the current delta rate.  And repeatedly decrease the gain by multiplying the negative delta rate if the error keeps getting lower.
5. When the new error from decrementing does not beat the previous best, I added back the delta rate gain so it stays at the gain with best error. Then I make the delta rate smaller, to be 0.9 of this cycle's delta rate. This delta rate will be used next time we tune the same parameters.
6. Then I move on to the next controller D and repeat steps 2 to 5, and then the I controller, and then go back to the P controller.

I used the following steps to actually getting to the values that I used for steering.

1. First, assign some reasonable values for P, I and D.  Since the simulation should not have much steering bias, if at all, I started out assigning I gain with a very small value (0.001), P and D have the same value (0.1).
2. Run twiddle with just a few steps, to make sure the car can drive on the first straight line without turning out of lane.
3. Then slowly increase the number of steps for simulation to include the first turn, and subsequent turns, and finally finishing the one lap.

Some manual tuning of parameters was also used. At one point, my car was driving on a curve even during a straight line. I then increased the D gain parameter aggressively to make the car drive more straight.  Also, some of the simulations that I ran gave very high values for both P and D.  Based on reading https://udacity-reviews-uploads.s3.amazonaws.com/_attachments/41330/1493863065/pid_control_document.pdf, I discovered that D value should not be too high for improving stability.

For the throttle, I keep the car to drive constantly at 40 mph.  The value that I chose seem to do the job so I did not do twiddling on this.

## Results

The following video records the result of the simulation finishes one lap.
https://youtu.be/SR4421bYRYU
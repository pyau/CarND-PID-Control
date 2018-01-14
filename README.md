# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## P, I, D components

The propotional component causes the car to steer propotionally to the lane center / CTE .  e.g, if the car is positioned far to the right, then P controller will steer the car to sharp left.  This is the main controller that steers the car in the correct direction.
[P omitted](https://github.com/pyau/CarND-PID-Control/blob/master/videos/P_omitted.mp4)

With P component omitted, the car never actually steer itself correctly towards the CTE.

The derivative controller prevents the propotional controller from overshooting.  Propotional controller has a tendency to overshoot, and derivative contoller will slow down the steering if the difference between current error and previous error is big, which points to oversteering.
[D omitted](https://github.com/pyau/CarND-PID-Control/blob/master/videos/D_omitted.mp4)

With D component omitted, the car quickly overshoots and runs itself out of the track.

The integral controller corrects the systemic bias that could cause P and D controllers to not approach CTE. In this project, however, the car can still finish a lap without using the I controller.

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
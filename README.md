# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
Aleksey Shepelev - October 09, 2017
   
### Path Generation
The path that is followed by the simulated vehicle consists of 50 points in global map coordinates. The vehicle moves from one point to the next every 20ms. The spacing between the points determines the vehicle's longitudinal velocity. The lateral offset of a point relative to the previous points determines the vehicle's yaw and lateral velocity. 

In the path planner that I have implemented, the path points follow a spline that is defined by 5 anchor points. Two of these anchor points are the previous points the vehicle has traversed and the remaining three are points on the anticipated trajectory 30, 60, and 90 meters away from the vehicle's present position. The vehicle's future position may be in a different lane and the spline anchor point equations include a lane number.

`vector<double> next_wp0 = getXY(car_s + 30, 2 + lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);`

Once the shape of the spline is defined, individual waypoints are placed along it to create a particular vehicle speed. We assume that the vehicle follows a constant angle from its current location to a point 30m ahead and calculate the spacing along the x-axis with this assumption. 

`double N = target_dist/(0.02*ref_vel/2.24); // Number of timesteps to cover target_dist`
`double x_point = x_add_on + target_x/N; // divide target_x into N steps`

The y positions of the waypoints are derived from the spline formula, as shown below:

`double y_point = s(x_point);`

As can be seen in the equations above, the spline can be modified by changing the reference velocity `ref_vel` and target lane `lane`. These parameters are set based on logic for lane keeping or changing lanes. 

The logic can be summarized as follows: "If my current lane does not have slower-moving traffic, stay in the current lane. If there is slower moving traffic ahead, pass this traffic (with a preference for passing on the left) by changing lanes, assuming the road  geometry and other vehicles allow a lane change to occur". This means that if the vehicle is able to pass without going outside the road boundaries and without merging straight into another vehicle, the vehicle will change lanes to pass. 

In addition for the logic that determines the target lane, some additional logic exists for the target velocity. If there is no slower moving traffic ahead, the velocity setpoint is slowly incremented until the speed limit is reached. Otherwise, the velocity is decremented until slower moving traffic has had a chance to advance to a reasonable distance in front. The vehicle will decrease speed only if it not able to merge to pass. 


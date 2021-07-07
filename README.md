# Self-Driving-Car-Artificial-Intelligence
![image](https://user-images.githubusercontent.com/49645682/124760152-bac79b80-df30-11eb-8a26-8227728754d7.png)

An artificial intelligence agent, that drives a car in a stochastic environment with 3 lanes, and other cars without hitting any car.

The agent works by calling the choose_next_state() function, which then calls make predictions (inside a for loop) so that it loops on each lane, to predict the car_s of each of the ego car, and all the other cars. Then the cost is calculated using distance_cost, velocity_cost, and turning_cost. After this the costs of the 3 lanes are put into array and then we calculate the minimum cost, if a lane is minimum, we then take a decision to turn the car by calling a function: Shoulditurn, which returns true or false after checking if there are cars at the lane we are turning to. If it returned true the car turns, if it returns false the car will slow down and re-evalute the situation. The agent is greedy as it plans short-term.

This is the cost function used for the distance, its negative as long as the x is higher than 30, when it reaches 30 or less it goes up exponentially.
![image](https://user-images.githubusercontent.com/49645682/124759166-a33be300-df2f-11eb-882e-6efb378c9502.png)

Link for the simulator: https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2

How to setup? using linux, put the main.cpp inside the path planner, and download the simulator, also setup uwebsocket, and then make and cmake (compile the files).

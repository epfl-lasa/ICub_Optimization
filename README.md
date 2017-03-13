# SNOPT
Here, we use SNOPT as a general solver for large scale non-linear optimization problem. One of the advantages of this solver is the ability to estimate unknown elements of gradient matrix. Also, it has different interfaces for C/C++/Fortran.

# Optimization Problem
Starting from a fixed initial configuration, the end-effector should move as fast as possible, toward a specific desired intercept point. So, the cost function is the velocity of end-effector in the next position. The variable of optimization is joint acceleration.
Also, we propose constraints on each joint angle and velocity to be in the feasible range. 
Another constraint is concerning the direction of velocity. In order to satisfy this constraint, the end-effector velocity in the next step, should be in the same direction with the vector from current position to desired position. Both vectors are normalized to have the same scale. 

# Prerequisite
In each iteration of the optimization loop, center of mass of the robot and velocity of the end-effector(directly or using the jacobian matrix) should be calculated. To this end, SD/FAST is implemented which is more efficient in computational time than using the Gazebo functions. So, first the model of robot should be run seperately in SD/FAST. For more information on how to use SD/FAST, please refer to [this repository](https://github.com/epfl-lasa/SDfast).

SNOPT library should aslo be provided in a seperate [folder](https://github.com/epfl-lasa/ICub_Optimization/tree/master/snopt). A detailed manual for SNOPT is available [here](web.stanford.edu/group/SOL/guides/sndoc7.pdf).

# Optimization Loop
The main loop which includes stopping criteria of the optimization has been defined in [test.cpp](https://github.com/epfl-lasa/ICub_Optimization/blob/master/icub-opt/test.cpp). The criteria (epsilon) is the difference between the end-effector position and desired intercept point. 

[W_STATIC.cpp](https://github.com/epfl-lasa/ICub_Optimization/blob/master/icub-opt/src/W_STATIC.cpp) includes two seperate functions (W_STATIC_init and W_STATIC_update) which define the initial condition, upper and lower bands on const function and constraints, and patern of non-zero elements of gradient matrix.

Also, there is another function (W_STATIC_shot) which define the equation of cost function and constrints. Also, known elements of gradient matrix should be defined here. 


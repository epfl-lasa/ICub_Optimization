# SNOPT

Here, we use SNOPT as a general solver for large scale non-linear optimization problem. One of the advantages of this solver is the ability to estimate unknown elements of gradient matrix. Also, it has different interfaces for C/C++/Fortran.

# Optimization Problem
Starting from a fixed initial configuration, the end-effector should move as fast as possible, toward a specific desired intercept point. So, the cost function is the velocity of end-effector in the next position. 
The variable of optimization is joint acceleration.
Also, we propose constraints on each joint angle and velocity to be in the feasible range. 
Another constraint is concerning the direction of velocity.




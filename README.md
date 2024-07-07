# Maneuvering Target State Estimation using Extended Kalman Filter (EKF)

## Description
This project implements an Extended Kalman filter to estimate the states and parameters of a target missile using a nonlinear point-mass model that includes gravitational and aerodynamic forces. Successful destruction of incoming enemy missile requires accurate estimation of its states. The EKF ensures accurate and reliable tracking, demonstrating its effectiveness in handling model non-linearities for target estimation. Two different point mass process models are used for ballistic and maneuvering target state estimatios. The effectiveness of estimator is shown with the help of simulation results.

## Implementation
The target was modelled using equations of motion, subsequently measurement model of Radar was used to obtain the measurements Range, Azimuth Angle, Elevation Angle, Range Rate. An Extended Kalman Filter (EKF) algorithm was then employed to estimate the position, velocity, drag coefficient and aerodynamic lift coefficients.

## How to use the files
1. **Target Modelling** 
    - The files Trajectory.m and Traj_Maneuvering.m contains the target modelling for ballistic and maneuvering target respectively.
    - Runge Kutta Integration of 4th-Order was employed with multiple iterations to obtain the true states of trajectory.
    - Run both of these files seperately to obtain the dataset of true parameters x, y, z, vx, vy, vz, beta, z1, z2 for ballistic and maneuvering target respectively. You will obtain two .txt files for the same namely Target_Trajectory.txt and Target_Trajectory_Man.txt which will be used further.

2. **Measurement Model**
    - The files Radar_Model.m and Radar_Model_Maneuv.m contains the Measurement Model using Radar for ballistic and maneuvering target respectively.
    - Run both of these files seperately to obtain the dataset of Measurements Range, \phi, \theta, Range Rate for ballistic and maneuvering target respectively.

3. **Extended Kalman Filter (EKF) Algorithm**
    - The files EKF.m, EKF1.m, EKF2.m contains the EKF algorithm for beta(dot) = 0 state for ballistic target, 1/beta(dot) = 0 state for ballistic target and 1/beta(dot) = 0 state for maneuvering target respectively.
    - Run these files to execute the EKF Algorithm and obtain the estimated states, estimated errors, covariance, innovation values and plot them as explained in next part.

4. **Simulation Plots**
    - The files Plot_Traj1.m, Plot_Traj2.m, Plot_Traj3.m contains the plot for estimated states vs true states, estimated errors & covariances, innovations & gains respectively.
    - Run the above files to obtain neccessary simulation results.

## Conclusion
An extended Kalman Filter based state estimation algorithm was formulated using line-of-sight angles, Range,
Range Rate measurements. The filter derivation employed non linear equations of motion for target dynamics
including gravitational and aerodynamic acceleration terms . A set of states and parameters required to estimate
the acceleration of target were employed in the filter formulation. Constant aerodynamic lift coefficients based
approach is used to model maneuvering target.

## Further Work
1. Further work includes implementation of Interacting Multiple Model (IMM) Filter using these two models to form a better estimator which can be used to improve integrated guidance-control systems.
2. Includes implementation of guidance-control algorithms for target interception.
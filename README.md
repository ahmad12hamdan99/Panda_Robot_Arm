# Introduction
This repo containes educational examples on Franka Emika 7 DOF robot using python and robtotics toolbox and numpy. 
We hope that it will be helpful for students and engineers to get a better understanding for topics like calibration and control.


<p align="center">
  <img src="HW1/img/banda.png" alt="Panda"/>
</p>


# Geometric Calibration

## Theoritical Background

From a manifacturing stand point it is extremely challenging to produce robots with zero erros in joints lenghts. Therefore each produced manipulator has small errors from the factory. Keep in mind all these small errors will affect the accuracy if not considered. It has been reported by a number of authors, the manipulator geometric errors are responsible for about 90% of the total positioning errors.


<p align="center">
  <img src="HW1/img/calibration_schema.png" alt="schema"/>
</p>
<p align = "center">
Fig.1.1 - The schematic of robot calibration procedure
</p>

1. The first step (modeling) is aimed at developing a suitable geometric model, which properly
describes the relation between the manipulator geometric parameters (link lengths and joint angles)
and the end-effector location (position and orientation).

2. The second step (design of experiments) is aimed at choosing optimal measurement
configurations. It should rely on an appropriate performance measure, which takes into account the
particularities of technological process. It should be also able to obtain solution within the work-cell
constraints, and to adjust the number of experiments based on the error estimation.

3. The third step (measurements) deals with carrying out calibration experiments using the
obtained configurations. Depending on the measurement methods (measurement tools and devices,
marker location, etc.), it may provide different experimental data (the end-effector position/location,
etc.).

4. At the fourth step (identification) the errors in geometric parameters are computed using the
corresponding model and proper identification algorithm. Using the experimental data, it is possible to
evaluate identification accuracy for the parameters of interest.

5. At the last step (implementation), the geometric errors are compensated by modification of
the geometric parameters embedded in the robot controller.






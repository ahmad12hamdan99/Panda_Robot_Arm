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



## Complete, irreducible geometric model
In this step we assum that the manipulator links are rigid enough and the non-geometric factors are negligible in this level of calibration, the general expression of the geometric model for a $n$ -dof serial manipulator can be described as a sequence of homogeneous transformations

$T(q) = T_{base}(  \pi_b) [ T_{joint}(q_1,   \pi_{q1}) . T_{link}(  \pi_{L1}) ] ... [ T_{joint}(q_n,   \pi_{qn}) . T_{link}(  \pi_{Ln}) ] T_{tool}(  \pi_t)$





where $T$ with different indices denote the relevant matrices of size $4 \times 4$ , $q$ is the vector of the actuated joint coordinates, while the vectors $π_b , π_t , π_{Lj}$ and the scalars $π_qj$ are the manipulator
geometric parameters corresponding to the base, tool, links and joints, respectively.

There are a number of techniques that allows us to obtain the manipulator model of such type, which is definitely complete but includes redundant parameters to be eliminated. 
In this work, we will use the model generation technique that is
based on dedicated analytical elimination rules and includes the following steps:

1. Construction of the complete and obviously reducible model in the form of
homogeneous matrices product.
* The base transformation  $T_{Base}=[T_x T_y T_z R_x R_y R_z]_b$
* The joint and link transformation $T_{joint,j} . T_{Link,j}$
  * for revolute joint $T_{joint,j} . T_{Link,j}= R_{e,j}(q_j,\pi_{qj}).[T_u T_v R_u R_v]_{Lj} $
  * for prismatic joint $T_{joint,j} . T_{Link,j}= T_{e,j}(q_j,\pi_{qj}).[R_u R_v ]_{Lj}$
* The tool transformation $T_{tool}=[T_x T_y T_z R_x R_y R_z]_t$

2. Elimination of non-identifiable and semi-identifiable parameters in accordance with specific rules for different nature and structure of consecutive joints.
* For the case of consecutive revolute joint $R_{e,j}(q_j,\pi_{qj})$
  * if $e_j \perp e_{j-1}$ , eliminate the term $R_{u,L_{j-1}}$ or $R_{v,L_{j-1}}$ that corresponds to $R_{e,j}$
  * if $e_j \parallel  e_{j-1}$ , eliminate the term $T_{u,L_{j-k}}$ or $T_{v,L_{j-k}}$ that defines the translation orthogonal to the joint axes, for which $k$ is minimum $( k \geq 1 )$


* For the case of consecutive prismatic joint $T_{e,j}(q_j,\pi_{qj})$
  * if $e_j \perp e_{j-1}$ , eliminate the term $T_{u,L_{j-1}}$ or $T_{v,L_{j-1}}$  that corresponds to $T_{e,j}$
  * if $e_j \parallel  e_{j-1}$ , eliminate the term $T_{u,L_{j-k}}$ or $T_{v,L_{j-k}}$ that defines the translation in the direction of axis $e_j$ , for which $k$ is minimum $( k \geq 1 )$

  Appling the previous Steps to our robot we got the following :
1. $T=[T_xT_yT_zR_xR_yR_z]_b . R_z(q_1+\Delta q_1) . [T_xT_yR_xR_y].R_y(q_2+\Delta q_2) . [T_xT_zR_xR_z].R_z(q_3+\Delta q_3).
2. [T_xT_yR_xR_y].R_y(q_4+\Delta q_4) .[T_xT_zR_xR_z] .R_z(q_5+\Delta q_5).[T_xT_yR_xR_y]. R_y(q_6+\Delta q_6). [T_xT_zR_xR_z]. R_z(q_7+\Delta q_7). [T_xT_yR_xR_y] . [T_xT_yT_zR_xR_yR_z]_t$

  * Applying the elimination rules : 
$T=[T_xT_yT_zR_xR_yR_z]_b.R_z(q_1+\cancel{\Delta q_1}).[T_xT_yR_x\cancel{R_y}].R_y(q_2+\Delta q_2).[T_xT_zR_x\cancel{R_z}].R_z(q_3+\Delta q_3).[T_xT_yR_x\cancel{R_y}].R_y(q_4+\Delta q_4).
[T_xT_zR_x\cancel{R_z}].R_z(q_5+\Delta q_5).[T_xT_yR_x \cancel{R_y}].R_y(q_6+\Delta q_6).[T_x\cancel{T_z}R_x\cancel{R_z}].R_z(q_7+\cancel{\Delta q_7}).[\cancel{T_x}\cancel{T_y}\cancel{R_x}\cancel{R_y}].[\cancel{T_x}\cancel{T_y}\cancel{T_z}\cancel{R_x}\cancel{R_y}\cancel{R_z}]_t$

  * the final equation :
$T_{robot}=R_z(q_1).[T_xT_yR_x].R_y(q_2+ \Delta q_2).[T_xT_zR_x].R_z(q_3+\Delta q_3).
[T_xT_yR_x].R_y(q_4+\Delta q_4).[T_xT_zR_x].R_z(q_5+\Delta q_5).[T_xT_yR_x].R_y(q_6+\Delta q_6).[T_xR_x].R_z(q_7)$

 

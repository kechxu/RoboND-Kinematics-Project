## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/simulation_console.png
[image2]: ./misc_images/arm_structure.png
[image3]: ./misc_images/wc_coordinates.png
[image4]: ./misc_images/result.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]Figure 1: Screenshot of the simulation console of forward kinematics.


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

##### 2.1 DH parameter table

The kinematic analysis can be found in the Figure 2. And the initial DH parameter table is defined in Table 1.
![alt text][image2]Figure 2: Coordinates and DH parameters.

$i$ | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$
--- | --- | --- | --- | ---
1 |        0 |      0 |  $d_1$ | $\theta_1$
2 | -$\pi/2$ |   $a_1$ |     0 | $\theta_2$-$\pi/2$
3 |        0 |   $a_2$ |     0 | $\theta_3$
4 | -$\pi/2$ |   $a_3$ |  $d_4$ | $\theta_4$
5 |  $\pi/2$ |      0 |     0 | $\theta_5$
6 | -$\pi/2$ |      0 |     0 | $\theta_6$
7 |        0 |      0 | $d_7$ | 0

Table 1: Initial DH parameter table. The parameters are to be determined.


According to the URDF file, we can derive the DH parameters as below.

$d_1 = Z_{joint1} + Z_{joint2} = 0.33 + 0.42 = 0.75$
$d_4 = X_{joint4} + X_{joint5} = 0.96 + 0.54 = 1.5$
$d_7 = X_{joint6} + X_{gripper} = 0.193 + 0.11 = 0.303$
$a_1 = X_{joint2} = 0.55$
$a_2 = Z_{joint3} = 1.25$
$a_3 = Z_{joint4} = -0.054$

Then, the DH parameter table can be updated as Table 2.

$i$ | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$
--- | --- | --- | --- | ---
1 |        0 |      0 |  0.75 | $\theta_1$
2 | -$\pi/2$ |   0.55 |     0 | $\theta_2$-$\pi/2$
3 |        0 |   1.25 |     0 | $\theta_3$
4 | -$\pi/2$ | -0.054 |  1.50 | $\theta_4$
5 |  $\pi/2$ |      0 |     0 | $\theta_5$
6 | -$\pi/2$ |      0 |     0 | $\theta_6$
7 |        0 |      0 | 0.303 | 0

Table 2: DH parameter table with the parameters calculated.

##### 2.1 Transform matrices

$$T_1^0 = \begin{pmatrix}\cos(\theta_1) & -\sin(\theta_1) & 0 & a_0 \\\
    \sin(\theta_1)\cos(\alpha_0) & \cos(\theta_1)\cos(\alpha_0) & -\sin(\alpha_0) & -\sin(\alpha_0)d_1 \\\
    \sin(\theta_1)\sin(\alpha_0) & \cos(\theta_1)\sin(\alpha_0) & \cos(\alpha_0) & \cos(\alpha_0)d_1 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_2^1 = \begin{pmatrix}\cos(\theta_2) & -\sin(\theta_2) & 0 & a_1 \\\
    \sin(\theta_2)\cos(\alpha_1) & \cos(\theta_2)\cos(\alpha_1) & -\sin(\alpha_1) & -\sin(\alpha_1)d_2 \\\
    \sin(\theta_2)\sin(\alpha_1) & \cos(\theta_2)\sin(\alpha_1) & \cos(\alpha_1) & \cos(\alpha_1)d_2 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_3^2 = \begin{pmatrix}\cos(\theta_3) & -\sin(\theta_3) & 0 & a_2 \\\
    \sin(\theta_3)\cos(\alpha_2) & \cos(\theta_3)\cos(\alpha_2) & -\sin(\alpha_2) & -\sin(\alpha_2)d_3 \\\
    \sin(\theta_3)\sin(\alpha_2) & \cos(\theta_3)\sin(\alpha_2) & \cos(\alpha_2) & \cos(\alpha_2)d_3 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_4^3 = \begin{pmatrix}\cos(\theta_4) & -\sin(\theta_4) & 0 & a_3 \\\
    \sin(\theta_4)\cos(\alpha_3) & \cos(\theta_4)\cos(\alpha_3) & -\sin(\alpha_3) & -\sin(\alpha_3)d_4 \\\
    \sin(\theta_4)\sin(\alpha_3) & \cos(\theta_4)\sin(\alpha_3) & \cos(\alpha_3) & \cos(\alpha_3)d_4 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_5^4 = \begin{pmatrix}\cos(\theta_5) & -\sin(\theta_5) & 0 & a_4 \\\
    \sin(\theta_5)\cos(\alpha_4) & \cos(\theta_5)\cos(\alpha_4) & -\sin(\alpha_4) & -\sin(\alpha_4)d_5 \\\
    \sin(\theta_5)\sin(\alpha_4) & \cos(\theta_5)\sin(\alpha_4) & \cos(\alpha_4) & \cos(\alpha_4)d_5 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_6^5 = \begin{pmatrix}\cos(\theta_6) & -\sin(\theta_6) & 0 & a_5 \\\
    \sin(\theta_6)\cos(\alpha_5) & \cos(\theta_6)\cos(\alpha_5) & -\sin(\alpha_5) & -\sin(\alpha_5)d_6 \\\
    \sin(\theta_6)\sin(\alpha_5) & \cos(\theta_6)\sin(\alpha_5) & \cos(\alpha_5) & \cos(\alpha_5)d_6 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_G^6 = \begin{pmatrix}\cos(\theta_7) & -\sin(\theta_7) & 0 & a_6 \\\
    \sin(\theta_7)\cos(\alpha_6) & \cos(\theta_7)\cos(\alpha_6) & -\sin(\alpha_6) & -\sin(\alpha_6)d_7 \\\
    \sin(\theta_7)\sin(\alpha_6) & \cos(\theta_7)\sin(\alpha_6) & \cos(\alpha_6) & \cos(\alpha_6)d_7 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

According to the difference in orientation in gripper coordinates frame

$$T_{gy} = \begin{pmatrix}\cos(-\pi/2) & 0 & \sin(-\pi/2) & 0 \\\
    0 & 1 & 0 & 0 \\\
    -\sin(-\pi/2) & 0 & \cos(-\pi/2) & 0 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_{gz} = \begin{pmatrix}\cos(\pi) & -\sin(\pi) & 0 & 0 \\\
    \sin(\pi) & \cos(\pi) & 0 & 0 \\\
    0 & 0 & 1 & 0 \\\
    0 & 0 & 0 & 1 \end{pmatrix}$$

Therefore, the whole transform matrix is

$$T_G^0 = T_1^0 \cdot T_2^1 \cdot T_3^2 \cdot T_4^3 \cdot T_5^4 \cdot T_6^5 \cdot T_G^6 \cdot T_{gy} \cdot T_{gz}$$


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### 3.1 Rotation matrices and wrist-center positions
$$R_{roll} = \begin{pmatrix}
1 & 0 & 0 \\\
0 & \cos(roll) & -\sin(roll) \\\
0 & \sin(roll) &  \cos(roll)
\end{pmatrix}$$

$$R_{pitch} = \begin{pmatrix}
\cos(pitch) & 0 & \sin(pitch) \\\
0 & 1 & 0 \\\
-\sin(pitch) & 0 & \cos(pitch)
\end{pmatrix}$$

$$R_{yaw} = \begin{pmatrix}
\cos(yaw) & -\sin(yaw) & 0 \\\
\sin(yaw) &  \cos(yaw) & 0 \\\
0 & 0 & 1
\end{pmatrix}$$

Then the roll-pitch-yaw rotation matrix is

$$R_{rpy} = R_{yaw} \cdot R_{pitch} \cdot R_{roll}$$

The above multiplication is because of extrinsic transform. Finally The coordinate of wrist center is

$$WC = \begin{pmatrix}p_x \\\ p_y \\\ p_z\end{pmatrix} - d_7R_{rpy}\begin{pmatrix}0 \\\ 0 \\\ 0\end{pmatrix}$$

![alt text][image3]Figure 3: Wrist center coordinates.

From Figure 3 and Figure 2, we can derive the parameters (Angles a, b, c and Sides A, B, C) by the coordinates of wrist center.

$A = \sqrt{d_4^2 + a_3^2} = \sqrt{1.5^2 + (-0.054)^2} = 1.501$

$B = \sqrt{(\sqrt{X_{wc}^2 + Y_{wc}^2} - 0.35)^2 + (Z_{wc}-0.75)^2}$

$C = a_2 = 1.25$

By Cosine Thoerem,

$\cos{a} = \frac{B^2+C^2-A^2}{2BC} \Rightarrow a = \arccos(\frac{B^2+C^2-A^2}{2BC})$

$\cos{b} = \frac{C^2+A^2-B^2}{2CA} \Rightarrow b = \arccos(\frac{C^2+A^2-B^2}{2CA})$

$\cos{c} = \frac{A^2+B^2-C^2}{2AB} \Rightarrow c = \arccos(\frac{A^2+B^2-C^2}{2AB})$

From Figure 2, we can compute the parameters

$\theta_1 = \arctan(\frac{Y_{wc}}{X_{wc}})$

$\theta_2 = \pi/2 - a + \arctan(\frac{Z_{wc}-d_1}{\sqrt{X_{wc}^2 + Y_{wc}^2} - a_1})$

$\theta_3 = \pi/2 - (b + \arctan(a_3/A))$

Using the Homogeneous RPY rotation from base link and gripper link, and the Tranpose of the Rotation matrix from link 0 to link 3, obtain $R_6^3$

$R_3^0 = T_1^0 \cdot T_2^1 \cdot T_3^2$

$R_6^3 = (R_3^0)^T \cdot R_{rpy}$

$$R_6^3 = \begin{pmatrix}
-\sin(\theta_4)\sin(\theta_6)+\cos(\theta_4)\cos(\theta_5)\cos(\theta_6) &
-\sin(\theta_4)\cos(\theta_6)-\sin(\theta_6)\cos(\theta_4)\cos(\theta_5) &
-\sin(\theta_5)\cos(\theta_4) \\\
\sin(\theta_5)\cos(\theta_6) & -\sin(\theta_5)\sin(\theta_6) & \cos(\theta_5) \\\
-\sin(\theta_4)\cos(\theta_5)\cos(\theta_6) - \sin(\theta_6)\cos(\theta_4) &
\sin(\theta_4)\sin(\theta_6)\sin(\theta_5) - \cos(\theta_4)\cos(\theta_6) &
\sin(\theta_4)\sin(\theta_5)
\end{pmatrix}$$

Then we can compute $\theta_4$, $\theta_5$, and $\theta_6$ using the elements in $R_6^3$

$\theta_4 = \arctan(\frac{\sin(\theta_4)\sin(\theta_5)}{\cos(\theta_4)\sin(\theta_5)}) = \arctan(\frac{R_6^3[3,3]}{-R_6^3[1,3]})$

$\theta_5 = \arctan\begin{pmatrix}\frac{\sqrt{(-\cos(\theta_4)\sin(\theta_5))^2 + (\sin(\theta_4)\sin(\theta_5))^2}}{\cos(\theta_5)}\end{pmatrix} = \arctan(\frac{\sqrt{(R_6^3[1,3])^2+(R_6^3[3,3])^2}}{R_6^3[2,3]})$

$\theta_6 = \arctan(\frac{\sin(\theta_6)}{\cos(\theta_6)}) = \arctan(\frac{-R_6^3[2,2]}{R_6^3[2,1]})$


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.  

The simulation showed that the trayectory was being drawn correctly, and the arm was following the trajectory. After some research and testing with some angle bounderies for the wrist joints, it was suggested on the course group that it may have been because of the API that performs the inverse transform of $R_3^0$ when obtaining $R_6^3$. Further I will test any other fast algorithm to solve the equations.

The following image shows that the algorithm implemented succeeds 8 out of 10 times on average.

![alt text][image4]Figure 4: Result of pick and place.



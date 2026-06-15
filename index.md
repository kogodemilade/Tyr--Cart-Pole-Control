
![Tyr](videos\cart-pole.gif)

The Cart-Pole Stabilization problem is a benchmark problem for control algorithms. The problem involves keeping a pendulum upright in the presence of disturbances and inherent instability.
This project is a simulation utilizing ROS2 and Gazebo Sim. This is intended to serve as an in-depth technical paper explaining the project, design choices and methodology.


This project can be split into 3 parts: the stl files creation, the matlab system design and the ros implementation. The CAD files were designed for quick simulation and can be 
found in the src\pendulum\description\meshes directory. The MATLAB system design and ROS implementation are discussed below.

<h2 align="center">MATLAB</h2>
<h3 align="center">Mathematical Modelling</h3>
![Cart-pole](images\cartpole.png)
<p align="center">(Image adopted from [Russ Tedrake's Underactuated Robotics](https://underactuated.mit.edu/) lecture notes)</p>


**Variables**

- $$l$$ = pendulum length
- $$m_c$$ = cart mass
- $$m_p$$ = pendulum mass
- $$x_c$$ = cart position
- $$x_p$$ = pendulum horizontal position
- $$y_p$$ = pendulum vertical position
- $$\dot{x}_c$$ = cart velocity
- $$\dot{x}_p$$ = pole horizontal velocity
- $$\dot{y}_p$$ = pole vertical velocity
- $$\theta$$ = pole angular displacement from vertical axis
- $$g$$ = acceleration due to gravity
- $$T$$ = Rod tension

First, we get the horizontal and vertical accelerations for the pendulum center of mass (which is at its tip):

$$x_p = x_c + l\sin\theta$$

$$\dot{x}_p = \dot{x}_c + l\dot{\theta}\cos\theta$$

$$\ddot{x}_p = \ddot{x}_c + l\ddot{\theta}\cos\theta - l\dot{\theta}^2\sin\theta \tag{1}$$

$$y_p = l\cos\theta$$

$$\dot{y}_p = -l\dot{\theta}\sin\theta$$

$$\ddot{y}_p = -l\ddot{\theta}\sin\theta - l\dot{\theta}^2\cos\theta$$

Applying Newton's laws:

$$\sum F_{xp} = m_p\ddot{x}_p = T\sin\theta \tag{2}$$

$$\sum F_{yp} = m_p\ddot{y}_p = T\cos\theta - m_p g$$

$$\sum F_{xc} = m_c\ddot{x}_c = F_x - T\sin\theta \tag{3}$$

Substituting equation (1) into equation (2):

$$m_p(\ddot{x}_c + l\ddot{\theta}\cos\theta - l\dot{\theta}^2\sin\theta) = T\sin\theta \tag{4}$$

$$m_p(-l\ddot{\theta}\sin\theta - l\dot{\theta}^2\cos\theta) = T\cos\theta - mg \tag{5}$$

Dividing equation (5) by equation (4) and cross-multiplying yields:

$$\cos\theta(\ddot{x}_c + l\ddot{\theta}\cos\theta - l\dot{\theta}^2\sin\theta) = \sin\theta(-l\ddot{\theta}\sin\theta - l\dot{\theta}^2\cos\theta) + mg\sin\theta$$

Simplifying:

$$\ddot{x}_c\cos\theta + l\ddot{\theta}\cos^2\theta = -l\ddot{\theta}\sin^2\theta + g\sin\theta$$

$$\ddot{x}_c\cos\theta = -l\ddot{\theta}(\cos^2\theta + \sin^2\theta) + g\sin\theta$$

From trig identities, $(\cos^2\theta + \sin^2\theta) = 1$

$$\therefore g\sin\theta - l\ddot{\theta} - \ddot{x}_c\cos\theta = 0 \tag{6}$$

Substituting equation (4) into equation (3):

$$m_c\ddot{x}_c = F_x - \ddot{x}_c m_p - m_p l\ddot{\theta}\cos\theta + m_p l\dot{\theta}^2\sin\theta$$

$$F_x = \ddot{x}_c(m_p+m_c) + \ddot{\theta}(m_p l\cos\theta) - m_p l\dot{\theta}^2\sin\theta \tag{7}$$

Putting the system of equations (6) and (7) in matrix form yields:

$$
\begin{bmatrix}
F_x \\
0
\end{bmatrix}
=
\begin{bmatrix}
m_p+m_c & m_p l\cos\theta \\
-\cos\theta & -l
\end{bmatrix}
\begin{bmatrix}
\ddot{x}_c \\
\ddot{\theta}
\end{bmatrix}
+
\begin{bmatrix}
-l m_p \dot{\theta}^2\sin\theta \\
g\sin\theta
\end{bmatrix}
$$

Making the (incomplete) state vector the subject:

$$
\begin{bmatrix}
\ddot{x}_c \\
\ddot{\theta}
\end{bmatrix}
=
\frac{1}{\Delta}
\begin{bmatrix}
-l & -m_p l\cos\theta \\
\cos\theta & m_p+m_c
\end{bmatrix}
\begin{bmatrix}
F_x + l m_p \dot{\theta}^2\sin\theta \\
-g\sin\theta
\end{bmatrix}
$$

Where $\Delta = -l(m_p + m_c) + m_p l\cos^2\theta$

Now, we can use the following small-angle approximations to linearize our system of equations:

$$\sin\theta \to \theta, \quad \cos\theta \to 1, \quad \dot{\theta}^2 \to 0 \quad \text{as} \quad \theta \to 0$$

$$
\begin{bmatrix}
\ddot{x}_c \\
\ddot{\theta}
\end{bmatrix}
=
\frac{1}{\Delta}
\begin{bmatrix}
-l & -m_p l \\
1 & m_p+m_c
\end{bmatrix}
\begin{bmatrix}
F_x \\
-g\theta
\end{bmatrix}
$$

This gives us the following linearized equations:

$$\ddot{x}_c = \frac{-m_p g}{m_c}\theta + \frac{1}{m_c} F_x$$

$$\ddot{\theta} = \frac{g(m_p+m_c)}{m_c l}\theta + \frac{-1}{m_c l}F_x$$

<h3 align="center">State-Space Representation</h3>
We choose the state vector to be the cart position, cart velocity, pole angle and pole angular velocity:

$$
\mathbf{x} =
\begin{bmatrix}
x_c \\
\dot{x}_c \\
\theta \\
\dot{\theta}
\end{bmatrix}
$$

The complete state differential equation becomes:

$$
\begin{bmatrix}
\dot{x}_c \\
\ddot{x}_c \\
\dot{\theta} \\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & \dfrac{-g m_p}{m_c} & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & \dfrac{g(m_p+m_c)}{m_c l} & 0
\end{bmatrix}
\begin{bmatrix}
x_c \\
\dot{x}_c \\
\theta \\
\dot{\theta}
\end{bmatrix}
+
\begin{bmatrix}
0 \\
\dfrac{1}{m_c} \\
0 \\
\dfrac{-1}{l m_c}
\end{bmatrix}
F_x
$$

<Matlab Programming and Simulink Simulation>
Now that we have the model, the next step is to model it in MATLAB and Simulink. One might have noticed that this model has been derived without friction terms. This is deliberate as friction terms are usually small enough that they are negligible, but they may be added in future releases. The code involves declaring the A (state transition matrix) and B (input matrix) matrices, and the C and D matrices (from the Output Equation in canonical state-space representation. For now, state observation techniques are not employed for simplicity while debugging, but will be integrated in future releases. Due to that, C amtrix is a 4x4 Identity matrix (meaning all states are measured directly). The D matrix is 0.
The parameters used are:
- $$l$$ = 1.2m
- $$m_c$$ = 2.0kg
- $$m_p$$ = 0.48kg
- $$g$$ =  $9.81 \, \text{ms}^{-2}$
- $$Q = diag([20 2 150 2])$$
- $$r = 0.1$$

where diag() represents a diagonal matrix with its values being the values on the matrix's main diagonal axis.

##Insert image of matlab code
![matlab](images/matlab.jpg)

Running the code produces the *Optimal Gains Vector*, K = 

The Simulink diagram is a direct implementation of canonical feedback stabilization, with some subtle additions. The output of the states are fed back as inputs to the states after passing through some gain matrix/vector. The diagram also includes 2 additions, which are random number generators multiplied by a unit step, to simulate sensor and process noise, to test the robustness of the controller. 
There are surely better ways to simulate random noise, but this seems to solve the problem.

##Insert image of simulink diagram
![simulink](images/simulink.jpg)

<h3 align="center"> Output over a 60 second time period</h3>
##Insert image of simulink output
![response](images/response.jpg)

<h3 align="center">Output without noise</h3>
##Insert image without noise
![de-noised response](images/denoise_response.jpg)



<h2 align="center">ROS Implementation</h2>
In the ROS implementation, 3 control algorithms are utilised: the PID control algorithm, LQR, and an energy-shaping swing up controller. All states are measured directly for simplicity, but in future releases, state acquisition would be more realistic, utilising a simulated imu sensor for angular velocity and an encoder or ToF sensor for cart distance, also artificially injecting adding gaussian noise. State observers would naturally be introduced, particularly the Luenberger Observer, Kalman Filter and Extended Kalman Filter as options to choose from. 

<h3 align="center">Swing Up Controller</h3>
The Swing up controller is necessary as a temporary controller that swings the controller upwards before lqr or pid takes over and stabilizes it near the equilibrium. This is required as the system is inherently nonlinear, and linearization assumptions break when states are too far from equilibrium conditions. The swing up controllers are algorithms capable of handling nonlinear systems, but due to 
some theoretical technicalities, they can not hold the pendulum in place. The swing up method chosen is *energy shaping*. This is deeply connected to Lyapunov control theory, and an explanation of why they can't stabilize the pendulum can be found in ![Chapter 9 of Russ Tedrake's Underactuated Robotics course](https://underactuated.mit.edu/lyapunov.html). After it swings it up to within the predesignated region ($$\pm{15\deg}$$, for example), the swing up controller hands authority over to the PID or LQR, depending on which is chosen.


<h3 align="center">PID Controller</h3>
The PID controller was manually tuned, but due to changes in pole geometry and weight, and cart weight, gains may need slight retuning. The system reached a sort of marginal stability with the cart and pole oscillating indefinitely within a small, fixed but noticeable region. The PID gains are:
$$k_p = 12.0$$
$$k_d = 1.0$$
$$k_i = 4.0$$
The result  of the signal having passed through the pid gains was multiplied by a system gain $$k=10x10^5$$.

##Insert image of pid block diagram
![pid_block](images/pid_block.jpg)


<h3 align="center">LQR Controller</h3>
The LQR controller was discussed in the MATLAB section. It had much better performance, especially when friction was dropped very low ($$10x10^{-5}$$). At such low values, PID failed to stabilize the system. This isn't to say LQR is inherently better than PID, as better PID tuning combinations may have created better stabilization. But, I do believe this exemplifies the elegance of LQR, as there is only little need for tuning, and most of the work is done once a model has been derived.

The Q matrix and r scalar are not so sensitive to change. The main consideration is ensuring the penalty for the pole angle state is much more than the rest. The cart position penalty should also be slightly bigger than the linear and angular velocity terms. If unfamiliar with LQR, the Underactuated Robotics course covers it in ![Chapter 8](https://underactuated.mit.edu/lqr.html).


<h3 align="center">Gazebo simulation</h3>
Ensuring the weights were as close to assumptions as possible was a bit tricky. For example, the conter of mass is assumed to be at the tip, with the pole being massless. This required me to create two separate STL files, one for the tip and the other for the pole, and using different materials to achieve that effect. I first considered making the pole's center of mass such that it corresponds with the height of the tip. This wouldn't have worked as there were still huge moment issues that persist. While the dimensions and weights do not correspond exactly with those stated in the matlab file, it falls in $$\pm{10%}$$ within the nominal values, which is tight enough that LQR can stabilize without any problems.
The swing up controller is still undergoing development and is not yet functional. Due to that, the gazebo launch and controller launch files are launched in quick succession to ensure the controller starts while the pole is still upright or near it. 

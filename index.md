# Tyr
A Cart-Pole Stabilization Project

![Tyr](videos\cart-pole.gif)

The Cart-Pole Stabilization problem is a benchmark problem for control algorithms. The problem involves keeping a pendulum upright in the presence of disturbances and inherent instability.
This project is a simulation utilizing ROS2 and Gazebo Sim. This is intended to serve as an in-depth technical paper explaining the project, design choices and methodology.


This project can be split into 3 parts: the stl files creation, the matlab system design and the ros implementation. The CAD files were designed for quick simulation and can be 
found in the src\pendulum\description\meshes directory. The MATLAB system design and ROS implementation are discussed below.

<a name=matlab><\a>
## MATLAB
### Mathematical Modelling 
![Cart-pole](images\cartpole.jpg)

**Variables**
**Variables**

- $l$ = pendulum length
- $m_c$ = cart mass
- $m_p$ = pendulum mass
- $x_c$ = cart position
- $x_p$ = pendulum horizontal position
- $y_p$ = pendulum vertical position
- $\dot{x}_c$ = cart velocity
- $\dot{x}_p$ = pole horizontal velocity
- $\dot{y}_p$ = pole vertical velocity
- $\theta$ = pole angular displacement from vertical axis
- $g$ = acceleration due to gravity, $9.81 \, \text{ms}^{-2}$
- $T$ = Rod tension

First, we get the horizontal and vertical accelerations for the pendulum center of mass (which is at its tip):

$$x_p = x_c + l\sin\theta$$

$$\dot{x}_p = \dot{x}_c + l\dot{\theta}\cos\theta$$

**Equation 1:**

$$\ddot{x}_p = \ddot{x}_c + l\ddot{\theta}\cos\theta - l\dot{\theta}^2\sin\theta$$

$$y_p = l\cos\theta$$

$$\dot{y}_p = -l\dot{\theta}\sin\theta$$

**Equation 2:**

$$\ddot{y}_p = -l\ddot{\theta}\sin\theta - l\dot{\theta}^2\cos\theta$$

Applying Newton's laws:

**Equation 3:**

$$\sum F_{xp} = m_p\ddot{x}_p = T\sin\theta$$

**Equation 4:**

$$\sum F_{yp} = m_p\ddot{y}_p = T\cos\theta - m_p g$$

**Equation 5:**

$$\sum F_{xc} = m_c\ddot{x}_c = F_x - T\sin\theta$$

Substituting Equation 1 into Equation 3:

**Equation 6:**

$$m_p(\ddot{x}_c + l\ddot{\theta}\cos\theta - l\dot{\theta}^2\sin\theta) = T\sin\theta$$

**Equation 7:**

$$m_p(-l\ddot{\theta}\sin\theta - l\dot{\theta}^2\cos\theta) = T\cos\theta - mg$$

Dividing Equation 7 by Equation 6 and cross-multiplying yields:

**Equation 8:**

$$\cos\theta(\ddot{x}_c + l\ddot{\theta}\cos\theta - l\dot{\theta}^2\sin\theta) = \sin\theta(-l\ddot{\theta}\sin\theta - l\dot{\theta}^2\cos\theta) + mg\sin\theta$$

Simplifying:

$$\ddot{x}_c\cos\theta + l\ddot{\theta}\cos^2\theta = -l\ddot{\theta}\sin^2\theta + g\sin\theta$$

$$\ddot{x}_c\cos\theta = -l\ddot{\theta}(\cos^2\theta + \sin^2\theta) + g\sin\theta$$

From trig identities, $(\cos^2\theta + \sin^2\theta) = 1$

**Equation 9:**

$$\therefore g\sin\theta - l\ddot{\theta} - \ddot{x}_c\cos\theta = 0$$

Substituting Equation 6 into Equation 5:

$$m_c\ddot{x}_c = F_x - \ddot{x}_c m_p - m_p l\ddot{\theta}\cos\theta + m_p l\dot{\theta}^2\sin\theta$$

**Equation 10:**

$$F_x = \ddot{x}_c(m_p+m_c) + \ddot{\theta}(m_p l\cos\theta) - m_p l\dot{\theta}^2\sin\theta$$

Putting the system of equations (Equation 9 and Equation 10) in matrix form yields:

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

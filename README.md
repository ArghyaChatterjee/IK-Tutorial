# Inverse Kinematic (IK) Tutorial

A tutorial for Inverse Kinematics (IK) for generic robots.

## Pink
In this tutorial, we will go through Pink IK software. The version of Pink used here is 3.5.0.

**P**ython **in**verse **k**inematics for articulated robot models, based on [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

![Banner for Pink v0.5.0](https://user-images.githubusercontent.com/1189580/192318997-ed7574c3-8238-451d-9548-a769d46ec03b.png)

## Installation

For best performance we recommended installing Pink from Conda:

```console
conda install -c conda-forge pink
```

You can also install it from PyPI:

```console
pip install pin-pink
```

## Usage

Pink solves differential inverse kinematics by [weighted tasks](https://scaron.info/robot-locomotion/inverse-kinematics.html). A task is defined by a *residual* function $e(q)$ of the robot configuration $q \in \mathcal{C}$ to be driven to zero. For instance, putting a foot position $p_{foot}(q)$ at a given target $p_{foot}^{\star}$ can be described by the position residual:

$$
e(q) = p_{foot}^{\star} - p_{foot}(q)
$$

In differential inverse kinematics, we compute a velocity $v \in \mathfrak{c}$ that satisfies the first-order differential equation:

$$
J_e(q) v = \dot{e}(q) = -\alpha e(q)
$$

where $J\_e(q) := \frac{\partial e}{\partial q}$ is the [task Jacobian](https://scaron.info/robotics/jacobian-of-a-kinematic-task-and-derivatives-on-manifolds.html). We can define multiple tasks, but some of them will come into conflict if they can't be all fully achieved at the same time. Conflicts are resolved by casting all objectives to a common unit, and weighing these normalized objectives relative to each other. We also include configuration and velocity limits, making our overall optimization problem a quadratic program:

$$
\begin{align}
\underset{v \in \mathfrak{c}}{\text{minimize}} \ & \sum_{\text{task } e} \Vert J_e(q) v + \alpha e(q) \Vert^2_{W_e} \\
\text{subject to} \ & v_{\text{min}}(q) \leq v \leq v_{\text{max}}(q)
\end{align}
$$

Pink provides an API to describe the problem as tasks with targets, and automatically build and solve the underlying quadratic program.

### Task costs

Here is the example of a biped robot that controls the position and orientation of its base, left and right contact frames. A fourth "posture" task, giving a preferred angle for each joint, is added for regularization:

```python
from pink.tasks import FrameTask, PostureTask

tasks = {
    "base": FrameTask(
        "base",
        position_cost=1.0,              # [cost] / [m]
        orientation_cost=1.0,           # [cost] / [rad]
    ),
    "left_contact": FrameTask(
        "left_contact",
        position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
        orientation_cost=0.0,           # [cost] / [rad]
    ),
    "right_contact": FrameTask(
        "right_contact",
        position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
        orientation_cost=0.0,           # [cost] / [rad]
    ),
    "posture": PostureTask(
        cost=1e-3,                      # [cost] / [rad]
    ),
}
```

Orientation (similarly position) costs can be scalars or 3D vectors. They specify how much each radian of angular error "costs" in the overall normalized objective. When using 3D vectors, components are weighted anisotropically along each axis of the body frame.

### Task targets

Aside from their costs, most tasks take a second set of parameters called *target*. For example, a frame task aims for a target transform, while a posture task aims for a target configuration vector. Targets are set by the `set_target` function:

```python
    tasks["posture"].set_target(
        [1.0, 0.0, 0.0, 0.0] +           # floating base quaternion
        [0.0, 0.0, 0.0] +                # floating base position
        [0.0, 0.2, 0.0, 0.0, -0.2, 0.0]  # joint angles
    )
```

Body tasks can be initialized, for example, from the robot's neutral configuration:

```python
import pink
from robot_descriptions.loaders.pinocchio import load_robot_description

robot = load_robot_description("upkie_description")
configuration = pink.Configuration(robot.model, robot.data, robot.q0)
for body, task in tasks.items():
    if type(task) is FrameTask:
        task.set_target(configuration.get_transform_frame_to_world(body))
```

A task can be added to the inverse kinematics once both its cost and target (if applicable) are defined.

### Differential inverse kinematics

Pink solves differential inverse kinematics, meaning it outputs a velocity that steers the robot towards achieving all tasks at best. If we keep integrating that velocity, and task targets don't change over time, we will converge to a stationary configuration:

```python
dt = 6e-3  # [s]
for t in np.arange(0.0, 42.0, dt):
    velocity = solve_ik(configuration, tasks.values(), dt, solver="quadprog")
    configuration.integrate_inplace(velocity, dt)
    time.sleep(dt)
```

If task targets are continuously updated, there will be no stationary solution to converge to, but the model will keep on tracking each target at best. Note that [`solve_ik`](https://stephane-caron.github.io/pink/inverse-kinematics.html#pink.solve_ik.solve_ik) will take care of both configuration and velocity limits read from the robot model.

## Examples

Illustrated examples showcase how Pink performs on various robot morphologies:

- Arm: [UR5](https://github.com/stephane-caron/pink/tree/main/examples#arm-ur5) and [UR5 with end-effector limits](https://github.com/stephane-caron/pink/tree/main/examples/barriers#arm-ur5)
- Dual arms: [Flying dual-arm UR3](https://github.com/stephane-caron/pink/tree/main/examples#flying-dual-arm-ur3)
- Dual arms: [Yumi with spherical self-collision avoidance](https://github.com/stephane-caron/pink/tree/main/examples/barriers#yumi-end-effector-self-collision-avoidance)
- Dual arms: [Iiwa with whole-body self-collision avoidance](https://github.com/stephane-caron/pink/tree/main/examples/barriers#iiwa-whole-body-collision-avoidance)
- Humanoid: [Draco 3](https://github.com/stephane-caron/pink/tree/main/examples#humanoid-draco-3)
- Mobile base: [Stretch R1](https://github.com/stephane-caron/pink/tree/main/examples#mobile-stretch)
- Quadruped: [Go2 squatting with floating-base limits](https://github.com/stephane-caron/pink/tree/main/examples/barriers#go2-squat)
- Wheeled biped: [Upkie rolling without slipping](https://github.com/stephane-caron/pink/tree/main/examples#wheeled-biped-upkie)

There are also more basic examples to get started:

- [Double pendulum](https://github.com/stephane-caron/pink/blob/main/examples/double_pendulum.py)
- [Loading a custom URDF](https://github.com/stephane-caron/pink/blob/main/examples/load_custom_urdf.py)
- [Visualization in MeshCat](https://github.com/stephane-caron/pink/blob/main/examples/visualize_in_meshcat.py)
- [Visualization in yourdfpy](https://github.com/stephane-caron/pink/blob/main/examples/visualize_in_yourdfpy.py)

Check out the [examples](https://github.com/stephane-caron/pink/tree/main/examples) directory for more.


## Description
### Differential Inverse Kinematics
#### Introduction
---
Inverse kinematics (IK) is the problem of computing a motion **q(t)** in the robot configuration space (e.g. joint angle coordinates) that achieves a desired motion in **task or workspace coordinates x(t)**. A task *i* can be, for instance, to put a foot on a surface, or to move the center of mass (CoM) of the robot to a target location. Tasks are then collectively defined by a set **x = (x₁, …, xₙ)** of points or frames attached to the robot, with for instance **x₁** the CoM, **x₂** the center of the right foot sole, etc.

The problems of forward and inverse kinematics relate configuration-space and workspace coordinates:

- **Forward kinematics**: compute workspace motions **x(t)** resulting from a configuration-space motion **q(t)**. Writing **FK** this mapping,

$$
x(t) = FK(q(t))
$$

- **Inverse kinematics**: compute a configuration-space motion **q(t)** so as to achieve a set of body motions **x(t)**. If the mapping **FK** were invertible, we would have

$$
q(t) = FK^{-1}(x(t))
$$

However, the mapping **FK** is not always invertible due to **kinematic redundancy**: there may be infinitely many ways to achieve a given set of tasks. For example, if there are only two tasks **x = (x_{lf}, x_{rf})** to keep left and right foot frames at a constant location on the ground, the humanoid can move its upper body while keeping its legs in the same configuration. In this post, we will see a common solution for inverse kinematics where redundancy is used achieve multiple tasks at once.

#### Kinematic task
---
Let us consider the task of bringing a point **p** located on one of the robot’s links, to a goal position **p\***, both point coordinates being expressed in the world frame. When the robot is in configuration **q**, the (position) residual of this task is:

$$
r(q) = p^* - p(q)
$$

The goal of the task is to bring this residual to zero. Next, from forward kinematics we know how to compute the Jacobian matrix of **p**:

$$
J(q) = \frac{\partial p}{\partial q}(q)
$$

which maps joint velocities **$\dot{q}$** to end-point velocities **$\dot{p}$** via

$$
J(q)\dot{q} = \dot{p}
$$

Suppose that we apply a velocity **$\dot{q}$** over a small duration **$\delta t$**. The new residual after **$\delta t$** is

$$
r' = r - \dot{p}\delta t
$$

Our goal is to cancel it, that is

$$
r' = 0 \;\Leftrightarrow\; \dot{p}\delta t = r
$$

which leads us to define the velocity residual:

$$
v(q, \delta t) \stackrel{\text{def}}{=} \frac{r(q)}{\delta t} = \frac{p^* - p(q)}{\delta t}
$$

The best option is then to select **$\dot{q}$** such that:

$$
J(q)\dot{q} = \dot{p} = v(q, \delta t)
$$

If the Jacobian were invertible, we could take

$$
\dot{q} = J^{-1}v
$$

However, that’s usually not the case (think of a point task where **J** has three rows and one column per DOF). The best solution that we can get in the least-square sense is the solution to:

$$
\min_{\dot{q}} \lVert J\dot{q} - v \rVert^2
$$

and is given by the pseudo-inverse **$J^\dagger$** of **J**:

$$
\dot{q} = J^\dagger v
$$

By writing this equivalently as

$$
(J^T J)\dot{q} = J^T v
$$

we see that this approach is exactly the **Gauss-Newton algorithm**. (There is a sign difference compared with the Gauss-Newton update rule, which comes from our use of the end-effector Jacobian **$\partial p / \partial q$** rather than the residual Jacobian **$\partial r / \partial q$**.)

#### Task gains
---
For this solution to work, the time step **$\delta t$** should be sufficiently small, so that the variations of the Jacobian term between **$q$** and **$q + \dot{q}\delta t$** can be neglected. The total variation is

$$
J(q + \dot{q}\delta t)\dot{q} - J(q)\dot{q} = \delta t\, \dot{q}^T H(q)\dot{q}
$$

where **$H(q)$** is the *Hessian* matrix of the task. This matrix is more expensive to compute than **$J$**. Rather than checking that the variation above is small enough, a *common practice* is to multiply the velocity residual by a proportional gain **$K_p \in [0, 1]$**:

$$
J(q)\dot{q} = K_p v
$$

For example, **$K_p = 0.5$** means that the system will (try at best to) cut the residual by half at each time step **$\delta t$**. Adding this gain does not change the exponential convergence to the solution **$r = 0$**, and helps avoid *overshooting* of the real solution. When you observe instabilities in your IK tracking, reducing task gains is usually a good idea.

#### Multiple tasks
---
So far, we have seen what happens for a single task, but redundant systems like humanoid robots need to achieve multiple tasks at once (moving feet from contact to contact, following with the center of mass, regulating the angular momentum, etc.) A common practice to combine tasks is weighted combination. We saw that a task *i* can be seen as minimizing $\lVert J_i \dot{q} - K_i v_i \rVert^2$. Then, by associating a weight $w_i$ to each task, the IK problem becomes:

$$
\min_{\dot{q}} \sum_{\text{task } i} w_i \lVert J_i \dot{q} - K_i v_i \rVert^2
$$

The solution to this problem can again be computed by pseudo-inverse or using a quadratic programming (QP) solver. This formulation has *some convergence properties*, but its solutions are always a *compromise* between tasks, which can be a problem in practice. For example, when a humanoid tries to make three contacts while it is only possible to make at most two, the solution found by a weighted combination will be somewhere “in the middle” and achieve no contact at all.

<div align="center">
  <img src="media/humanoid.png" width="400">
</div>


In practice, we can set implicit priorities between tasks by having e.g. $w_i = 10^4$ for the most important task, $w_j = 10^2$ for the next one, etc. This is for instance how the **pymanoid** IK is used in **this paper**. This solution emulates the behavior of a **hierarchical quadratic program (HQP)** (unfortunately, there has been no open source HQP solver available as of 2016–2021). See this **talk by Wieber (2015)** for a deeper overview of the question.

#### Inequality constraints

Last but not least, we need to enforce a number of inequality constraints to avoid solutions that violate e.g. joint limits. The overall IK problem becomes:

$$
\min_{\dot{q}} \sum_{\text{task } i} w_i \lVert J_i \dot{q} - K_i v_i \rVert^2
$$

subject to

$$
\dot{q}^- \le \dot{q} \le \dot{q}^+
$$

Inequalities between vectors being taken componentwise. This time, the pseudo-inverse solution cannot be applied (as it doesn’t handle inequalities), but it is still possible to use a QP solver, such as **CVXOPT** or **quadprog** in Python. These solvers work on problems with a quadratic cost function and linear inequalities:

$$
\min_x \; (1/2)\, x^T P x + r^T x
$$

subject to

$$
Gx \le h
$$

The weighted IK problem falls under this framework.

##### Cost function

Consider one of the squared norms in the task summation:

$$
\lVert J_i \dot{q} - K_i v_i \rVert^2
= (J_i \dot{q} - K_i v_i)^T (J_i \dot{q} - K_i v_i)
$$

$$
= \dot{q}^T J_i^T J_i \dot{q} - K_i v_i^T J_i \dot{q} - K_i \dot{q}^T J_i^T v_i + K_i^2 v_i^T v_i
$$

$$
= \dot{q}^T (J_i^T J_i)\dot{q} - 2 (K_i v_i^T J_i)\dot{q} + K_i^2 v_i^T v_i
$$

where we used the fact that, for any real number $x$, $x^T = x$. As the term in $v_i^T v_i$ does not depend on $\dot{q}$, the minimum of the squared norm is the same as the minimum of $(1/2)\dot{q}^T P_i \dot{q} + r_i^T \dot{q}$ with $P_i = J_i^T J_i$ and $r_i = -K_i v_i^T J_i$. The pair $(P, r)$ for the weighted IK problem is finally $P = \sum_i w_i P_i$ and $r = \sum_i w_i r_i$.

#### Joint limits
---
There are two types of joint limits we can take into account in first-order differential inverse kinematics: joint velocity and joint-angle limits.

Since we solve a problem in **$\dot{q}$**, velocity limits can be directly written as **$G\dot{q} \le h$** where **$G$** stacks the matrices **$+E_3$** and **$-E_3$**, while **$h$** stacks the corresponding vectors **$\dot{q}^+$** and **$-\dot{q}^-$**. In practice, joint velocities are often symmetric, so that **$\dot{q}^+ = \dot{q}_{\max}$** and **$\dot{q}^- = -\dot{q}_{\max}$**, and **$h$** stacks **$\dot{q}_{\max}$** twice.

Next, we want to implement limits on joint angles **$q^- \le q \le q^+$** so that the robot stays within its mechanical range of motion. A solution for this is to add velocity bounds:

$$
\dot{q}^- = K_{\text{lim}} \frac{q^- - q}{\delta t}
$$

$$
\dot{q}^+ = K_{\text{lim}} \frac{q^+ - q}{\delta t}
$$

where **$K_{\text{lim}} \in [0, 1]$** is a proportional gain. For example, a value of **$K_{\text{lim}} = 0.5$** means that a joint angle update will not exceed half the gap separating its current value from its bounds (**Kanoun, 2011**).

#### To go further

The differential inverse kinematics formulation we have seen here is implemented in Python in **Pink** (based on Pinocchio) as well as in **mink** (based on MuJoCo) by Kevin Zakka. Both libraries comes with a number of examples for manipulators, humanoids, quadrupeds, robotic hands, ... Alternatively to forming a quadratic program (with worst-case complexity cubic in the number of parameters), it is also possible to solve differential IK directly on the kinematic tree with linear time-complexity. This alternative is implemented in the **LoIK** open-source library. It solves problems faster than QP solvers, but it only applies to single-body tasks (e.g. no center-of-mass task).

#### Levenberg-Marquardt damping

One efficient technique to make the IK more numerically stable is to add **Levenberg-Marquardt damping**, an extension of Thikonov regularization where the damping matrix is chosen proportional to the task error. This strategy is implemented e.g. by the **BodyTask** of Pink. See (**Sugihara, 2011**) for a deeper dive.

#### Second-order differential IK

Differential IK at the acceleration level (second order) has been more common than first order version we have seen in this note. It can be found for instance in the **Tasks** library, which powered ladder-climbing by an **HRP-2 humanoid robot**, as well as in the **TSID** open-source library.

#### History of the expression “inverse kinematics”

Thirty-five years ago (**Richard, 1981**), “inverse kinematics” was defined as the problem of finding joint angles **q** fulfilling a set of tasks **g(q) = 0**, which is a fully geometric problem. (A better name for it could have been "inverse geometry", considering that *kinematics* which means motion and that this problem does not involve motion.) Various solutions were proposed to approach this question, the most widely reproduced idea being to compute successive velocities that bring the robot, step by step, closer to fulfilling **g(q) = 0**. This approach was called “differential inverse kinematics” (**Nakamura, 1990**) to distinguish it from the existing “inverse kinematics” name. (A better name for it could have been, well, "inverse kinematics", since it does involve motion!)



## Frequently Asked Questions

- [Can I solve **global** inverse kinematics?](https://github.com/stephane-caron/pink/discussions/66#discussioncomment-8224315)
- [Can I make velocities smoother?](https://github.com/stephane-caron/pink/discussions/103)
- [My configuration gets stuck somewhere and does not solve the task, what is going on?](https://github.com/stephane-caron/pink/discussions/66#discussioncomment-8224315)

## Global inverse kinematics

Pink implements differential inverse kinematics, a first-order algorithm that converges to the closest optimum of its cost function. It is a **local** method that does not solve the more difficult problem of [global inverse kinematics](https://github.com/stephane-caron/pink/discussions/66). That is, it may converge to a global optimum, or to a local one stuck to some configuration limits. This behavior is illustrated in the [simple pendulum with configuration limit](https://github.com/stephane-caron/pink/blob/main/examples/simple_pendulum_configuration_limit.py) example.


Don't forget to add yourself to the BibTeX above and to `CITATION.cff` if you contribute to this repository.

## See also

Software:

- [mink](https://github.com/kevinzakka/mink): differential inverse kinematics in Python, based on the MuJoCo physics engine.
- [Jink.jl](https://github.com/adubredu/Jink.jl): Julia package for differential multi-task inverse kinematics.
- [PlaCo](https://github.com/rhoban/placo): C++ inverse kinematics based on Pinocchio.
- [pymanoid](https://github.com/stephane-caron/pymanoid): precursor to Pink based on OpenRAVE.
- [TSID](https://github.com/stack-of-tasks/tsid): C++ inverse kinematics based on Pinocchio.

Technical notes:

- [Inverse kinematics](https://scaron.info/robotics/inverse-kinematics.html): a general introduction to differential inverse kinematics.
- [Jacobian of a kinematic task and derivatives on manifolds](https://scaron.info/robotics/jacobian-of-a-kinematic-task-and-derivatives-on-manifolds.html).
- [Control Barrier Functions](https://simeon-ned.com/blog/2024/cbf/).


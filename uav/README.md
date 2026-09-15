# UAV — PPO Reinforcement Learning

This directory contains the **PPO (Proximal Policy Optimization)** implementation for continuous-control reinforcement learning of the quadrotor in Project Falcon.

The current implementation consists of a custom, dependency-light quadrotor simulation environment and a Stable-Baselines3 PPO training pipeline.

## Files

```text
uav/
├── README.md
├── train.py
└── quad_env.py
```

### `quad_env.py`

Custom Gymnasium environment implementing the quadrotor dynamics and RL interface.

The environment models:

* 6-DOF rigid-body dynamics
* Quaternion-based attitude representation
* X-configuration 4-rotor mixer
* Linear and angular drag
* Rotor thrust-based control
* Hover and waypoint tasks
* Optional domain randomization

The current observation vector contains 13 states:

```text
[x, y, z,
 vx, vy, vz,
 qw, qx, qy, qz,
 wx, wy, wz]
```

The action vector contains four normalized rotor commands:

```text
[T1, T2, T3, T4]
```

with each action normalized to:

```text
[-1, 1]
```

and subsequently mapped to the available rotor thrust range.

The environment currently uses a simplified physics model without ROS or PyBullet, allowing rapid RL experimentation before transferring the policy to a higher-fidelity simulator.

---

### `train.py`

PPO training entrypoint for the quadrotor hover and waypoint tasks.

The training script supports:

* Stable-Baselines3 PPO
* Parallel/vectorized environments
* Observation normalization
* Reward normalization
* TensorBoard logging
* Periodic checkpoints
* Evaluation callbacks
* Reproducible random seeds
* Domain randomization

Available tasks:

```text
hover
waypoint
```

Default training configuration:

```text
Timesteps:       2,000,000
Parallel envs:   8
Max steps/env:   500
Learning rate:   3e-4
Batch size:      256
Epochs:          10
Gamma:           0.99
GAE lambda:      0.95
Clip range:      0.2
```

The training script uses an MLP policy with separate two-layer networks for the policy and value function.

---

## Environment

The current environment supports two tasks.

### Hover

The quadrotor is initialized near 1 m altitude and is required to stabilize around a target hover position.

The default hover target is:

```text
[0.0, 0.0, 1.5] m
```

### Waypoint

A waypoint is randomly generated within the simulation workspace.

The target position is sampled in:

```text
x ∈ [-1.5, 1.5]
y ∈ [-1.5, 1.5]
z ≥ 0.5 m
```

This allows the policy to learn point-to-point flight rather than only fixed-position hovering.

---

## Dynamics

The quadrotor uses a simplified rigid-body model.

Current physical parameters include:

```text
Mass                  = 0.5 kg
Gravity               = 9.81 m/s²
Arm length            = 0.17 m
Thrust-to-weight      = 2.5
Yaw torque coefficient = 0.02
Linear drag           = 0.15
Angular drag          = 0.02
```

The rotor thrusts are converted into total thrust and body torques using an X-configuration mixer:

```text
[T_total, τx, τy, τz]ᵀ = M[T1, T2, T3, T4]ᵀ
```

## The resulting forces and torques are propagated using semi-implicit Euler integration.

## Reward Function

The current reward combines:

* Alive bonus
* Position error
* Linear velocity penalty
* Angular velocity penalty
* Action-rate penalty
* Uprightness reward

The reward is currently structured as:

```text
R =
    alive bonus
  - position error
  - velocity penalty
  - angular velocity penalty
  - action-rate penalty
  + uprightness reward
```

A crash is triggered when the vehicle:

* Drops below 0.05 m altitude
* Moves more than 5 m from the target
* Becomes inverted

A significant terminal penalty is then applied.

---

## PPO Training Pipeline

The current pipeline is:

```text
        ┌──────────────────┐
        │ QuadrotorEnv     │
        │ Gymnasium        │
        └────────┬─────────┘
                 │
                 ▼
        ┌──────────────────┐
        │ Vectorized Envs  │
        │ Dummy/Subproc    │
        └────────┬─────────┘
                 │
                 ▼
        ┌──────────────────┐
        │   VecNormalize   │
        │ Obs + Reward     │
        └────────┬─────────┘
                 │
                 ▼
        ┌──────────────────┐
        │    PPO MLP       │
        │  Actor + Critic  │
        └────────┬─────────┘
                 │
                 ▼
        ┌──────────────────┐
        │ Rotor Commands   │
        └────────┬─────────┘
                 │
                 ▼
        ┌──────────────────┐
        │ Quadrotor Physics│
        └──────────────────┘
```

Multiple environments can be executed in parallel using `SubprocVecEnv`, while a single environment uses `DummyVecEnv`. Observation and reward normalization are applied using `VecNormalize`.

---

## Training

Basic hover training:

```bash
python train.py --task hover
```

Waypoint training:

```bash
python train.py --task waypoint
```

Custom training duration:

```bash
python train.py \
    --task hover \
    --timesteps 2000000
```

Enable domain randomization:

```bash
python train.py \
    --task hover \
    --domain_randomize
```

Increase parallel environments:

```bash
python train.py \
    --task waypoint \
    --n_envs 16
```

The script stores TensorBoard logs under:

```text
./logs
```

and model checkpoints under:

```text
./checkpoints
```

The final PPO model and `VecNormalize` statistics are also saved after training.

---

### To Do
* [ ] Custom Gymnasium quadrotor environment
* [ ] 6-DOF rigid-body dynamics
* [ ] Quaternion attitude representation
* [ ] Four-rotor mixer
* [ ] Continuous 4-dimensional action space
* [ ] 13-dimensional observation space
* [ ] Hover task
* [ ] Waypoint task
* [ ] Reward function
* [ ] PPO training pipeline
* [ ] Vectorized environment support
* [ ] Observation/reward normalization
* [ ] Checkpointing
* [ ] Evaluation callback
* [ ] TensorBoard logging
* [ ] Optional domain randomization
* [ ] Validate PPO convergence on hover task
* [ ] Tune reward weights
* [ ] Evaluate waypoint tracking performance
* [ ] Analyze policy stability and control smoothness
* [ ] Improve domain randomization
* [ ] Add more challenging flight scenarios
* [ ] Introduce disturbances and external forces
* [ ] Evaluate generalization to unseen targets
* [ ] Integrate moving-target scenarios
* [ ] Connect policy to ROS 2
* [ ] Port the environment/policy to Gazebo or PX4 SITL
* [ ] Evaluate sim-to-real transfer

The current environment was intentionally designed so that its observation/action contract can be transferred to higher-fidelity simulation environments such as Gazebo/PX4 or other quadrotor simulators.

---

## Intended Falcon Integration

The long-term pipeline is:

```text
                UAV Sensors
                     │
                     ▼
              ┌─────────────┐
              │    ROS 2    │
              └──────┬──────┘
                     │
                     ▼
              State Extraction
                     │
                     ▼
              ┌─────────────┐
              │ PPO Policy  │
              └──────┬──────┘
                     │
              Control Commands
                     │
                     ▼
              ┌─────────────┐
              │ PX4 / Gazebo│
              └─────────────┘
```

The current implementation is the low-level RL development stage. Higher-fidelity simulation, ROS 2 integration, PX4 integration, and UAV–UGV coordination will be added as the policy and environment mature.

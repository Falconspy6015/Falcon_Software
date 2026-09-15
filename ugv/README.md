# UGV — TD3 Reinforcement Learning

This directory contains the **TD3 (Twin Delayed Deep Deterministic Policy Gradient)** implementation for continuous-control reinforcement learning of the **UGV (Unmanned Ground Vehicle)** in Project Falcon.

## Directory Structure

```text
ugv/
├── README.md
│
├── td3/
│   ├── actor.py
│   ├── critic.py
│   ├── td3.py
│   └── networks.py
│
├── environment/
│   ├── ugv_env.py
│   ├── observations.py
│   └── rewards.py
│
├── training/
│   ├── train.py
│   └── config.yaml
│
├── evaluation/
│   └── evaluate.py
│
├── checkpoints/
│
└── utils/
    ├── replay_buffer.py
    └── logger.py
```

The exact structure may evolve as the RL pipeline is integrated with the simulation and ROS 2 stack.

---

## RL Algorithm

The controller uses **TD3**, an off-policy actor–critic algorithm designed for continuous action spaces.

TD3 addresses common issues in vanilla DDPG through:

1. **Twin Critics**

   Two independent Q-functions are maintained:

   $$
   Q_{\theta_1}(s,a), \qquad Q_{\theta_2}(s,a)
   $$

   The smaller estimate is used for the target:

   $$
   y = r + \gamma(1-d)
   \min_{i=1,2} Q_{\theta_i'}(s',a')
   $$

2. **Delayed Policy Updates**

   The actor is updated less frequently than the critics to improve training stability.

3. **Target Policy Smoothing**

   Noise is added to target actions to reduce exploitation of narrow Q-function peaks.

---

## UGV Control

The policy is intended for **continuous UGV control**.

A preliminary action space may be:

```text
Action:
    linear_velocity
    angular_velocity
```

or, depending on the simulated vehicle model:

```text
Action:
    left_wheel_velocity
    right_wheel_velocity
```

The final action representation will be determined by the UGV dynamics and ROS 2 control interface.

---

## Observation Space

The observation vector is expected to contain information relevant to navigation and control.

Potential observations include:

```text
UGV pose
UGV linear velocity
UGV angular velocity
Target relative position
Target relative heading
Obstacle distances
Local occupancy / lidar information
Previous action
```

The observation space will be refined based on the selected simulation environment and control objective.

---

## Reward Function

The reward function will encourage the UGV to reach its target efficiently while maintaining safe operation.

Possible components include:

### Target Progress

Reward movement toward the target:

$$
r_{progress} =
d_{t-1} - d_t
$$

where \(d_t\) is the current distance to the target.

### Heading Alignment

Reward alignment between the UGV heading and target direction.

### Collision Penalty

A significant negative reward is applied for collisions or unsafe proximity to obstacles.

### Goal Reward

A positive terminal reward is provided when the UGV reaches the target.

### Control Penalty

A small penalty may be applied to excessive control effort or abrupt changes in velocity.

A preliminary reward can therefore be expressed as:

$$
R =
w_pR_{progress}
+w_hR_{heading}
+w_gR_{goal}
-w_cR_{collision}
-w_uR_{control}
$$

The reward weights will be tuned experimentally.

---

## Environment

The TD3 agent will initially be trained in simulation.

The intended pipeline is:

```text
Gazebo
   │
   │ State / Sensor Data
   ▼
ROS 2
   │
   ▼
UGV Environment
   │
   ▼
TD3 Agent
   │
   │ Continuous Action
   ▼
ROS 2 Control
   │
   ▼
Gazebo UGV
```

The environment should expose a standard RL interface:

```python
observation = env.reset()

next_observation, reward, done, info = \
    env.step(action)
```

---

## Dynamic Obstacles

A key requirement of the UGV controller is operation in environments containing **dynamic obstacles**.

The RL controller should therefore be evaluated under scenarios such as:

* Static obstacles
* Moving pedestrians
* Randomly appearing obstacles
* Changing obstacle trajectories
* Narrow passages
* Target movement
* Temporary path blockage

The TD3 controller is intended to complement, rather than necessarily replace, classical path-planning and replanning methods.

Potential planners under investigation include:

* D* Lite
* Theta* Lite
* Incremental Theta*
* Incremental Phi*

The division between global planning, local planning, and RL-based control will be determined during system integration.

---

## Training Pipeline

The expected training loop is:

```text
Initialize Actor
Initialize Twin Critics
Initialize Target Networks
Initialize Replay Buffer

        │
        ▼

Reset UGV Environment

        │
        ▼

Observe State

        │
        ▼

Actor → Continuous Action

        │
        ▼

Execute Action in Simulation

        │
        ▼

Receive:
    next_state
    reward
    done

        │
        ▼

Store Transition
in Replay Buffer

        │
        ▼

Sample Mini-Batch

        │
        ▼

Update Critics

        │
        ▼

Delayed Actor Update

        │
        ▼

Soft Target Updates

        │
        └──────────► Repeat
```

---

## ROS 2 Integration

The trained policy will eventually interface with the Falcon ROS 2 stack.

A conceptual interface is:

```text
             ┌─────────────────┐
             │  Sensor / State  │
             │      Nodes       │
             └────────┬────────┘
                      │
                      ▼
             ┌─────────────────┐
             │   UGV ROS 2     │
             │  State Extractor│
             └────────┬────────┘
                      │
                      ▼
             ┌─────────────────┐
             │   TD3 Policy    │
             └────────┬────────┘
                      │
                v, ω / Wheel Cmd
                      │
                      ▼
             ┌─────────────────┐
             │ UGV Controller  │
             └────────┬────────┘
                      │
                      ▼
                  Gazebo UGV
```

The ROS 2 interface and custom message definitions will be finalized during integration.

---

## Training vs Deployment

Training and deployment should remain separated.

### Training

```text
Gazebo → ROS 2 → RL Environment → TD3
```

Training may use:

* Randomized environments
* Domain randomization
* Exploration noise
* Large replay buffers
* Frequent checkpointing

### Deployment

```text
Real / Simulated Sensors
          ↓
       ROS 2
          ↓
    Trained TD3 Actor
          ↓
    UGV Controller
```

## Evaluation Metrics

The controller should be evaluated using:

| Metric                       | Description                                   |
| ---------------------------- | --------------------------------------------- |
| Success Rate                 | Percentage of episodes reaching the target    |
| Collision Rate               | Percentage of episodes resulting in collision |
| Episode Reward               | Cumulative reward                             |
| Goal Distance                | Final distance from target                    |
| Path Length                  | Distance travelled by UGV                     |
| Time to Goal                 | Time required to reach target                 |
| Control Smoothness           | Magnitude/frequency of control changes        |
| Robustness                   | Performance under unseen environments         |
| Dynamic Obstacle Performance | Success under moving obstacles                |

---

## Current Status

**Status: In Development**

## Work to be done

* [ ] Implement TD3 Actor network
* [ ] Implement twin Critic networks
* [ ] Implement replay buffer
* [ ] Implement target-network updates
* [ ] Implement UGV Gym-style environment
* [ ] Define observation and action spaces
* [ ] Develop and tune reward function
* [ ] Integrate Gazebo simulation
* [ ] Integrate ROS 2 state/action interfaces
* [ ] Add dynamic obstacle scenarios
* [ ] Compare TD3 against classical local controllers
* [ ] Evaluate generalization to unseen environments
* [ ] Integrate with UAV–UGV coordination pipeline
---

## References

* Fujimoto, H., van Hoof, H., & Meger, D. — **Addressing Function Approximation Error in Actor-Critic Methods**, 2018.
* ROS 2 Documentation
* Gazebo Documentation

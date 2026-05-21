---
name: "reinforcement-learning-control"
description: "Reinforcement learning for robotic control. Invoke when designing RL environments, reward functions, or training agents for autonomous excavator operation."
---

# Reinforcement Learning Control

Guidelines for training and deploying Reinforcement Learning (RL) agents for the continuous control of the excavator's hydraulics and movements.

## When to Invoke
- When designing Gym/Gymnasium environments for the excavator simulation.
- When defining reward functions based on digging efficiency, energy consumption, or target angle tracking.
- When integrating algorithms like PPO, SAC, or TD3 for continuous control spaces.

## Best Practices
1. **Environment Setup**: Define the state space using the 4 joint angles (boom, arm, bucket, swing) and the action space using the hydraulic analog values and swing duration.
2. **Sim-to-Real Transfer**: Train policies in simulation (e.g., Isaac Sim or Gazebo) and apply domain randomization (friction, mass, hydraulic delay) before testing on the real hardware.
3. **Safety Constraints**: Always wrap the RL agent's output with the `AngleController`'s joint limits (`min_angle` and `max_angle`) to prevent hardware damage during exploration or exploitation.
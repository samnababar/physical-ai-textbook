---
sidebar_position: 4
---

# Week 4: Deploying VLAs on Robots

Once a Vision-Language-Action model is trained, the next step is to deploy it on a physical robot. This is often referred to as the "inference" phase.

## The Inference Loop

The process of running a VLA on a robot can be broken down into the following steps, which are executed in a loop:

1.  **Perceive:** The robot captures an image from its camera.
2.  **Localize (Optional but Recommended):** The robot determines its own state, such as the position of its joints and its base.
3.  **Get Command:** The robot receives a natural language command from a user.
4.  **Preprocess:** The image and language command are preprocessed and tokenized in the same way they were during training.
5.  **Infer Action:** The preprocessed inputs are fed into the trained VLA, which autoregressively generates a sequence of action tokens.
6.  **Post-process:** The predicted action tokens are converted back into continuous motor commands (e.g., joint velocities or target poses).
7.  **Execute:** The motor commands are sent to the robot's controllers for execution.
8.  **Repeat:** The loop repeats, allowing the robot to react to changes in the environment and execute long-horizon tasks.

## Challenges in Deployment

Deploying VLAs on real-world robots presents several challenges:

-   **Latency:** The entire inference loop must be fast enough for the robot to operate in real-time. This often requires hardware acceleration (e.g., a GPU on the robot).
-   **The "Sim2Real" Gap:** Models trained in simulation may not perform well in the real world due to differences in physics, lighting, and sensor noise. Techniques like domain randomization, which we discussed in Chapter 3, are crucial for bridging this gap.
-   **Safety:** The robot must be able to operate safely around humans and in unstructured environments. This may involve adding safety-critical checks and balances on top of the VLA's output.
-   **Open vs. Closed Vocabulary:** An action decoder can be either "open vocabulary" or "closed vocabulary".
    -   **Closed Vocabulary:** The model can only predict from a predefined set of actions it was trained on. This is safer but less flexible.
    -   **Open Vocabulary:** The model can potentially generate novel combinations of actions. This is more flexible but harder to train and less predictable.

## The Future is Embodied

The development of Vision-Language-Action models represents a major step towards creating truly intelligent and general-purpose robots. As these models become more capable and robust, they will unlock a new era of human-robot interaction.

Congratulations on completing this textbook! You have journeyed from the basic building blocks of ROS 2 to the cutting edge of AI-driven robotics. The future of humanoid robotics is bright, and you are now equipped with the foundational knowledge to be a part of it.
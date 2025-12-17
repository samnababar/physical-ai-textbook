---
sidebar_position: 3
---

# Week 3: Training VLAs

Training a Vision-Language-Action model is a complex process that requires a large amount of demonstration data. The most common training paradigm is **Behavior Cloning (BC)**.

## Behavior Cloning

In behavior cloning, the VLA is trained to directly imitate expert demonstrations. The model is given the same visual input and language command that the expert received, and its goal is to output the same sequence of actions that the expert performed.

This is a form of *supervised learning*, where the "correct" output (the expert's actions) is known. The model is trained to minimize the difference between its predicted actions and the expert's actions.

## Data Collection

The biggest challenge in training VLAs is collecting a large and diverse dataset of expert demonstrations. This typically involves:

1.  **Teleoperation:** A human operator controls the robot using a joystick or other input device to perform a variety of tasks.
2.  **Data Logging:** During teleoperation, the robot's sensor data (camera images), the operator's commands (which are transcribed into language), and the robot's motor actions are all recorded.

This process is repeated thousands or even millions of times to create a dataset that covers a wide range of scenarios.

### Example Data Point in a VLA Dataset

A single data point in a behavior cloning dataset might look like this:

-   **Vision Input:** A sequence of camera images from the robot's perspective.
-   **Language Input:** A text string, e.g., "place the apple in the bowl."
-   **Action Output (the "label"):** A sequence of motor commands (e.g., end-effector poses) that correspond to the actions taken by the human expert.

## The Training Process

1.  **Data Preprocessing:** The collected data is preprocessed and tokenized. Images are converted into tokens by the vision encoder, text is tokenized by the language encoder, and robot actions are discretized into a vocabulary of action tokens.
2.  **Model Training:** The VLA is trained using a standard deep learning framework like PyTorch or TensorFlow. The model is fed the vision and language tokens and trained to predict the corresponding action tokens.
3.  **Loss Function:** A cross-entropy loss function is typically used to measure the difference between the model's predicted action tokens and the ground truth action tokens from the expert demonstration.
4.  **Optimization:** An optimizer like Adam is used to update the model's weights to minimize the loss.

This process is repeated for many epochs until the model's performance on a validation set stops improving.

![Data collection for robotics](https://images.squarespace-cdn.com/content/v1/5ab1338d31d4df3633390978/1678832669399-W04L2N9D1PAG35D62R2Y/RT-1-3.gif)
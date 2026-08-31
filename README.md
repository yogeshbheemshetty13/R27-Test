# R27 Test

<p align="center">
  <img src="https://github.com/teamrudra/r26_test/blob/main/misc/rover.webp" width="480" height="480"/>

#### Some Instructions
1. You may use any online resources, datasheets, or documentation needed, but be mindful of your time and stay focused on the task.
2. The duration of the test is 90 mins from 5:15pm to 6:45 pm.
3. There will be a MCQ test conducted [here](https://rudra-test.vercel.app/)
4. There are 5 tasks in the tests. Complete all of them.
5. In case you are not able to complete all the tasks, do upload whatever you are able to.
6. In the `README.md` of your repository include your thought process, places where you got stuck, where you used the help of AI, google or other online resources.
7. Even if you are not able to solve anything, do fill the README and what your thought process would have been.
8. Carefully read the instructions to implement the required functionality.
9. Install a c compiler and [git](https://git-scm.com/downloads) if you haven't already done it.
10. After finishing your test, provide the link to your forked repository in the google form provided at the end.

### Aim/Objective: To build a communication system that safely transfers, processes and decodes messages between threads and uses the received coordinates to control a rover. 

## Description
This test evaluates your ability to understand, debug, and implement functionality in an existing C-based embedded/robotics code-base. The test has five tasks. You are given an existing code-base with partially implemented functionality. Your task is to understand the code, identify the issues, implement the required changes, and verify your solution.

### Task 0: Fork the provided repository and ensure it is set to PUBLIC so we can access and assess your work.
### Task 1: Implement encoding and decoding functions for embedded communication.
Fixed incorrect logic in the encoder and decoder, handled buffer limits and edge-case inputs, and verify that decoding returns the original data.
### Task 2: Manage multi-threading and synchronization using POSIX threads.
Review and correct the existing implementation of mutexes, semaphores, and message queues while maintaining the core architecture.
### Task 3: Control a differential-drive rover to navigate to a target.
Complete the drive-to-target functionality to calculate direction, generate appropriate left/right wheel velocities, and handle heading wraparound.
### Task 4: Compile and run the code.
Verify the workflow on the provided rover simulator and ensure the project compiles successfully.

#### Code
1. [src/main.c](src/main.c): Code for running the multi-threading and synchronization architecture.
2. [src/en_dc.c](src/en_dc.c): Rectify errors in this code to correctly encode/decode data and handle invalid inputs.
3. [lib/en_dc.h](lib/en_dc.h): Header file containing declarations for the encoding and decoding logic.
4. [src/queue.c](src/queue.c): Correct the existing message queue implementation.
5. [src/mutex.c](src/mutex.c): Review and fix POSIX mutex and semaphore logic.
6. [src/drive.c](src/drive.c): Complete the defined `drive_to_target()` function to guide the rover.
7. [lib/drive.h](lib/drive.h): Header file containing parameters and declarations for rover control.

## Build the project:

(make sure you are in the root directory)

```
cmake -S . -B build
```

```
cmake --build build --verbose
```

```
./build/queue_test
```

To clean the build:

```
rm -rf build
```

# Solution
## Understanding
Describe what you understood about the problem.

## Thought Process
After understanding the problem, describe how you decided to proceed towards solving the question. Also document any use of external resources or AI tools. 

## Implementation
How did you decide to implement your solution.

**Good luck!**
# Google Form
https://forms.gle/A8CaByv4ohfrCmmWA

<p align="center">
  <img src="https://github.com/teamrudra/r25-test/blob/main/datasheets/feynman-simple.jpg" width="600" height="600"/>
</p>
## AI Assistance

AI tools were used to help understand the C concepts, identify implementation issues, and assist with debugging. The implementation was reviewed and tested using the provided test cases and GitHub Actions.

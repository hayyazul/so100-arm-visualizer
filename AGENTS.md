
# System Notes
 - You are running on WSL in a windows machine. 
 - This has an Intel iRIS GPU. Do not try any GPU programming magic, this is just info to keep in mind (so you don't try to use CUDA or any incompatible code).

# Very Important Practices (DO NOT IGNORE)
 - Always plan out your code first.
 - ALWAYS use git.
    - When adding a new feature, branch to it first then develop. When we are done with a feature, I will assist in merging to main. Within a branch, you may run commands as needed (git add, git commit, etc. You may also create and merge sub-branches to the branch you are working on, but do not merge to main unless I explicitly tell you to do so).
 - Always type hints everywhere, even for temporary or one off variables. Even for test files. For literally and unconditionally everything.
 - Comment your code. Do not use emojis. 
    - If a function or class is forced to be dependent on another function or class, reference that function which or class which is used (does not apply to library functions). 
    - Tersity is preferred, but not at the cost of clarity. 
    - Always update comments.
 

# General Practices
 - Remember to always use the venv environment and to exit the pre-existing standard conda environment.
    - Run "conda deactivate" to exit the pre-existing standard conda environment, then "source venv/bin/activate" to enter the venv environment.
 - Have strict error handling.
 - If using non-universally defined abreviations, add a comment at the top of the file, function or class explaining it. You don't need to do this for commonly known things like "i" or "eeg", but if someone who is new to the codebase can't understand it immedately it is not common enough.
 - Always make both unit tests and human-interface tests (the latter is more fluid and interactable, it is used to test the system as a whole).
 - Always make sure that the code is modular and easy to understand. Remember these principals in helping with that:
    - Functions should do one thing, they should do it well, they should do it only.
       - Do not make too many functions; deep implementations with simple interfaces are always better.
    - Classes are similar, but in terms of responsibilities rather than actions. Try not to have too many classes, and try to make them as cohesive as possible.
    - Pull complexity downwards.
 - Try to find existing implementations and libraries.
 - SCRATCHPAD.md is there for you to jot down info, things which didn't work, etc. It is open ended and there for you to use as needed.
    - This document should not exceed a page in length.
 - One-off or utility scripts should be kept in a separate folder called "scripts".

# Useful tools and libraries
 - LeRobot: https://github.com/lerobot-dev/lerobot
 - uv: Has a faster version of pip, just prepend "uv" to any pip command.
 - git: Version management. Never merge to main unless I say so.

# MVP: 
    - An inverse kinematics solver for the so100 robotic arm.

# Phase 1:
    - A system that can take in the desired end-effector position and orientation and output the joint angles required to reach that position and orientation.

# Phase 2:
    - A simple visualizer of the given system. It need not be fancy or even interactable at this stage.
        - Maybe you can use Manim.

# Phase 3:
    - An interactable version of the visualizer; the user should be able to control the end-effector position and orientation and see the arm move in real-time in the visualizer.
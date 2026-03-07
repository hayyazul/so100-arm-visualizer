
# General Practices
 - Remember to always use the venv environment and to exit the pre-existing standard conda environment.
 - Always plan out your code first.
 - Always use type hints everywhere, even for temporary or one off variables.
 - Comment your code. Do not use emojis. 
    - If a function or class is forced to be dependent on another function or class, reference that function which or class which is used (does not apply to library functions). 
    - Tersity is preferred, but not at the cost of clarity. 
    - Always update comments.
 - Have strict error handling.
 - If using non-universally defined abreviations, add a comment at the top of the file, function or class explaining it. You don't need to do this for commonly known things like "i" or "eeg", but if someone who is new to the codebase can't understand it immedately it is not common enough.
 - Always make both unit tests and human-interface tests (the latter is more fluid and interactable, it is used to test the system as a whole).
 - Always make sure that the code is modular and easy to understand. Remember these principals in helping with that:
    - Functions should do one thing, they should do it well, they should do it only.
       - Do not make too many functions; deep implementations with simple interfaces are always better.
    - Classes are similar, but in terms of responsibilities rather than actions. Try not to have too many classes, and try to make them as cohesive as possible.
    - Pull complexity downwards.
 - Try to find existing implementations and libraries.

# MVP: 
    - An inverse kinematics solver for the so100 robotic arm.

# Phase 1:
    - A system that can take in the desired end-effector position and orientation and output the joint angles required to reach that position and orientation.

# Phase 2:
    - A simple visualizer of the given system. It need not be fancy or even interactable at this stage.
        - Maybe you can use Manim (check if this is a good or bad idea first, then implement the best solution).

# Phase 3:
    - An interactable version of the visualizer; the user should be able to control the end-effector position and orientation and see the arm move in real-time in the visualizer.
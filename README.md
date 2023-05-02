# RBE 598 OM-X DirectedResearch
Jack Rothenberg (jarothenberg@wpi.edu)
Jatin Kohli (jpkohli@wpi.edu)

We experimented with the OpenManipulator-X arm to test new features for future versions of RBE 3001. We also modified the existing lab documents and provided solution code for use with the OM-X arm. Releases have been made for the RBE 3001 starter code and solution files for each lab.

## Branches
- master: Experiment Code
  - torque: Script that uses PID based on position error and current control to follow a task space trajectory. The constants were found experimentally and motion plots for several sets of constants are provided.
  - statics: Script to calculate the torque from the motor currents and use that to determine the change in torque when an object is held. It then lists a given number of objects by their weights.
  - manipulability: Scripts to calculate the manipulability using svd and sqrt(det(J\*J^T)). The eigenvectors and eigenvalues of the J\*J^T are used to plot the manipulability ellipsoid on a stick model with the robot's pose in real time.
- lab1-starter: Starter code for Lab1, to be given at the start of 3001
- lab1: Solution code for Lab1
- lab2: Solution code for Lab2
- lab3: Solution code for Lab3
- lab4: Solution code for Lab4
- lab5: Solution code for Lab5, including statics extra credit based on statics code

## Links
### Download MATLAB SDK
https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

### OpenManipulator-X Documentation
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/

### Dynamixel XM430-W350 Documentation
https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ 

### Dynamixel Wizard 2.0
https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/

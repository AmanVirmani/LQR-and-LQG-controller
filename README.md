# LQR-and-LQG-controller
Implementation of LQR and LQG conttroller in Matlab for controlling a crane with two supsended pendulums. 

The problem statement solved in this project has two major components and solutions have been provided in **Report.pdf**

## First Component
The system includes of a crane that moves along an one-dimensional track. It behaves as a
friction-less cart with mass M actuated by an external force F that constitutes the input of
the system. There are two loads suspended from cables attached to the crane. The loads
have mass m1 and m2, and the lengths of the cables are l1 and l2, respectively.

<p align="center">
<img src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/crane.png">
Figure 1: System Under Consideration
</p>

A) Obtain the equations of motion for the system and the corresponding nonlinear state-space
representation.


B) Obtain the linearized system around the equilibrium point specified by x = 0 and θ 1 =
θ 2 = 0. Write the state-space representation of the linearized system.


C) Obtain conditions on M, m 1 , m 2 , l 1 , l 2 for which the linearized system is controllable.


D) Choose M = 1000Kg, m 1 = m 2 = 100Kg, l 1 = 20m and l 2 = 10m. Check that the system is controllable and obtain an LQR controller. Simulate the resulting response to initial conditions when the controller is applied to the linearized system and also to the original nonlinear system. Adjust the parameters of the LQR cost until you obtain a suitable response. Use Lyapunov’s indirect method to certify stability (locally or globally) of the closed-loop system.

## Second Component 

E) Suppose that you can select the following output vectors: x(t), (θ 1 (t), θ 2 (t)), (x(t), θ 2 (t)) or (x(t), θ 1 (t), θ 2 (t)).
Determine for which output vectors the linearized system is observable.


F) Obtain your "best" Luenberger observer for each one of the output vectors for which the system is observable and simulate its response to initial conditions and unit step input. The simulation should be done for the observer applied to both the linearized system and the original nonlinear system.

G) Design an output feedback controller for your choice of the "smallest" output vector. Use the LQG method and apply the resulting output feedback controller to the original nonlinear system. Obtain your best design and illustrate its performance in simulation. How would you reconfigure your controller to asymptotically track a constant reference on x ? Will your design reject constant force disturbances applied on the cart ?

## Outputs

<img align="left" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/C1_linear.jpg">
<img align="right" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/C1_non_linear.jpg">
<img align="left" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/C3_linear.jpg">
<img align="right" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/C3_non_linear.jpg">
<img align="left" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/C4_linear.jpg">
<img align="right" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/C4_non_linear.jpg">
<img align="left" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/LQR_linear.jpg">
<img align="right" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/LQR_non_linear.jpg">
<img align="center" width="400" height="400" src="https://github.com/AmanVirmani/LQR-and-LQG-controller/blob/master/LQG_non_linear.jpg">

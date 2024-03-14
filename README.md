# Project Report: Modeling Chaotic Motion in Coupled Spring-Mass System

### Introduction
The problem addressed in this project is the modeling of chaotic motion in a coupled spring-mass system. Chaotic motion arises in systems where small changes in initial conditions lead to vastly different outcomes over time. This phenomenon has significant implications in various scientific fields, including physics, engineering, and mathematics. Understanding chaotic behavior is crucial for predicting the behavior of complex systems accurately.

### Objective
The objective of this project is to develop a numerical model to simulate the chaotic motion of a coupled spring-mass system. The model aims to explore how variations in mass, spring constants, and initial conditions affect the system's behavior over time. By studying chaotic motion in this system, we seek to deepen our understanding of nonlinear dynamics and provide insights into real-world applications.

### Methodology
The numerical technique employed in this project is the Fourth-Order Runge-Kutta (RK4) method. This method is widely used for solving ordinary differential equations (ODEs) numerically with high accuracy. The RK4 method involves approximating the solution at each time step by considering the slopes at various points within the interval. By iteratively updating the state variables, the RK4 method enables the simulation of dynamic systems accurately.

Assumptions made in the numerical model include:
- Negligible damping effects
- Small angles approximation for the pendulum motion
- Linear spring behavior within the operating range

### Shortcomings
Despite its effectiveness, the numerical model has several limitations:
- Neglects damping effects, which could significantly impact the system's behavior in real-world scenarios.
- Relies on linear approximations for spring behavior, limiting its applicability to systems with nonlinear springs.
- Assumes small angles for pendulum motion, which may not hold for large amplitude oscillations

### Contribution of Group Members
Each member of the group contributed to the project in the following ways:
- Member 1: Implemented the RK4 numerical solver and developed the main simulation loop.
- Member 2: Designed the graphical visualization of the spring-mass system and integrated it into the simulation.
- Member 3: Conducted energy analysis of the system, including kinetic and potential energy calculations, and plotted energy graphs.

### Input-Output sample
Enter required values in the command window
- Enter mass m1: ____
- Enter mass m2: ____
- Enter initial length l01: ____
- Enter initial length l02: ____
- Enter spring constant k1: ____
- Enter spring constant k2: ____
- Enter simulation time: ____

##### Here's sample screenshot(Input)
![input sample](https://github.com/ta-r-ek/Matlab_Project/assets/162335400/376f5d23-4fd6-45fb-b8ed-d5e60d1d210b)

##### Output simulation plot - 
![output simulation](https://github.com/ta-r-ek/Matlab_Project/assets/162335400/3cb4ab65-ccc7-4337-8f36-93075cc0975c)

##### Kinetic Energy vs Time plot -
![energy plot](https://github.com/ta-r-ek/Matlab_Project/assets/162335400/611abbb7-5221-4d66-b831-2ff78067114a)

# Furuta Pendulum State Feedback Control

This repository documents the regulator control design stage for a rotary inverted pendulum (Furuta pendulum) developed during the Minor in Autonomy of Unmanned Aerial Vehicles at Tecnológico de Monterrey.

This work builds directly on the previous state-space modeling practice and focuses on the design and implementation of a full-state feedback controller for the pendulum in its inverted configuration. The practice includes the selection of desired closed-loop poles from performance specifications, controllability verification, computation of the state feedback gain matrix, and evaluation of the closed-loop response on the real system.

Different methods were used to compute and validate the gain matrix, including MATLAB pole placement, transformation-based methods, and the companion matrix method. The results show that the controller successfully stabilizes the rotary inverted pendulum, while also highlighting practical limitations such as residual steady-state error and deviations from the desired overshoot due to modeling inaccuracies and real-world dynamics.

This repository is intended to present the practical implementation of state feedback control for an underactuated system and to show the transition from mathematical modeling to real-time controller design and validation.

## Topics Covered
- Rotary inverted pendulum control
- Furuta pendulum stabilization
- Full-state feedback control
- Pole placement design
- Controllability analysis
- Gain matrix computation
- Companion matrix method
- MATLAB implementation
- Experimental validation

## Tools Used
- MATLAB
- Simulink
- QUARC
- Rotary inverted pendulum platform

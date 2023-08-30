# Feedback Linearization-based Receding Horizon Control for KheperaIV
Code developed for the work published in "C. Tiriolo, G. Franzè, W. Lucia -  A Receding Horizon Trajectory Tracking Strategy for Input-Constrained Differential-Drive Robots via Feedback Linearization - TCST 2022"
Full paper at: ******


# Trajectory Tracking Problem Formulation 
Consider a **Khepera IV** differential-drive robot subject to input constraints, i.e., limits on the admissible wheels' angular velocities, and a bounded trajectory $r(t)$. Design a feedback control law $[\omega_{R}(t),\omega_{L}(t)]^T=\phi(\left[p_c(t)^T,\theta(t)\right]^T,r(t))$ such that the tracking error $\xi(t)=r(t)-p_c(t)$ is bounded.

# Prerequisites 
The code was tested on Matlab 2020a environment and it requires a Khepera IV robot (see https://www.k-team.com/khepera-iv) to run. 
The code implements a Bluetooth client that sends velocity commands to the robot in order to make it track a desired trajectory (eight-shaped and circular tractories are currently available). For further details refer to the paper.


# File Descriptions 
- Khepera_iv_FL_RHC_traj_track.m: It is the main script to run in order to perform the experiment proposed in the paper. 
- Khepera4.m: It represents the core of the application. It is a Matlab class that implements the main communication functionalities between the tracking controller running on Matlab and the server running on Khepera IV
- STTT_control_parameters.m: It's a Matlab class defining the parameters needed by the proposed tracking controller.
- "eight_traj_generation.m" and "circular_traj_generation.m" implements the reference trajectory, an eight-shaped and a circular one, respectively.

# Demo 
- Connect KheperaIV to the host machine through Bluetooth and set the right port on the script "Khepera_iv_FL_RHC_traj_track.m".
- Run the Bluetooth server on the KheperaIV side and then, run the script Khepera_iv_FL_RHC_traj_track.m

# Video 
- https://www.youtube.com/watch?v=A0Tlbgr08tY&ab_channel=PreCySeGroup
- https://www.youtube.com/watch?v=L3wmg-pHx_4&list=PLh-6B_s-jPuT8RTDOJM96GXu4y1IBIeoC&ab_channel=PreCySeGroup

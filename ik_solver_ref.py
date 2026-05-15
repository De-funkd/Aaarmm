#!/usr/bin/env python3
"""
Interactive IK Solver - Matches Assignment Expected Output Format
Black Coffee Robotics Take-Home Assignment

Usage:
    python p_solver_custom.py
    
Then enter target position when prompted.
"""

import numpy as np
import pinocchio as pin
import os
import sys


class InteractiveIKSolver:
    """
    Interactive wrapper for the IK solver.
    Provides the exact interface shown in the assignment example.
    """
    
    def __init__(self, urdf_path):
        """Initialize the IK solver with URDF model."""
        # Load the robot model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # Get model name from URDF
        self.model_name = self.model.name
        
        # Get end-effector frame
        self.end_effector_id = self.model.getFrameId("end_effector")
        
        # Initial joint configuration (all zeros)
        self.initial_q = np.zeros(self.model.nq)
        
        print(f"model name: {self.model_name}")
        print()
        print(f"initial joint angles: {self.initial_q.tolist()}")
    
    def forward_kinematics(self, q):
        """Compute forward kinematics."""
        pin.framesForwardKinematics(self.model, self.data, q)
        ee_pose = self.data.oMf[self.end_effector_id]
        return ee_pose.translation
    
    def solve_ik(self, target_position, max_iter=1000, tol=1e-4):
        """
        Solve inverse kinematics using CLIK algorithm.
        
        Args:
            target_position: Desired [x, y, z] position
            max_iter: Maximum iterations
            tol: Convergence tolerance
            
        Returns:
            Joint configuration that reaches target
        """
        target_position = np.array(target_position)
        
        # Start from initial configuration
        q = self.initial_q.copy()
        
        # CLIK parameters
        dt = 0.1
        damping = 1e-6
        
        for iteration in range(max_iter):
            # Compute current position
            pin.framesForwardKinematics(self.model, self.data, q)
            current_pose = self.data.oMf[self.end_effector_id]
            current_position = current_pose.translation
            
            # Calculate error
            error = target_position - current_position
            error_norm = np.linalg.norm(error)
            
            # Check convergence
            if error_norm < tol:
                return q, True
            
            # Compute Jacobian
            J = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.end_effector_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            
            # Position Jacobian (first 3 rows)
            J_pos = J[:3, :]
            
            # Damped pseudo-inverse
            J_pinv = J_pos.T @ np.linalg.inv(
                J_pos @ J_pos.T + damping * np.eye(3)
            )
            
            # Update configuration
            dq = J_pinv @ error
            q = pin.integrate(self.model, q, dq * dt)
        
        # Did not converge
        return q, False
    
    def run_interactive(self):
        """
        Run interactive mode - prompts user for target position.
        Matches the expected output format from assignment.
        """
        # Prompt for input
        user_input = input("Enter the desired poisition (e.g., 0.5, 0.0, 0.5): ")
        
        # Parse input
        try:
            coords = [float(x.strip()) for x in user_input.split(',')]
            if len(coords) != 3:
                print("Error: Please provide exactly 3 coordinates (x, y, z)")
                return
            
            target_position = np.array(coords)
            
        except ValueError:
            print("Error: Invalid input format. Please use: x, y, z")
            return
        
        # Solve IK
        joint_angles, converged = self.solve_ik(target_position)
        
        if converged:
            print("Convergence achieved!!")
            print()
            print(f"Final Joint Angles: {joint_angles.tolist()}")
        else:
            print("Warning: IK did not fully converge")
            print(f"Final Joint Angles: {joint_angles.tolist()}")
            
            # Verify how close we got
            final_pos = self.forward_kinematics(joint_angles)
            error = np.linalg.norm(final_pos - target_position)
            print(f"Position error: {error:.6f}m")


def main():
    """Main entry point for interactive IK solver."""
    # Path to URDF file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "robot_arm.urdf")
    
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found at {urdf_path}")
        sys.exit(1)
    
    # Create solver
    solver = InteractiveIKSolver(urdf_path)
    
    # Run interactive mode
    solver.run_interactive()


if __name__ == "__main__":
    main()

"""
WERR 22-DoF Kinematic Reflex Engine (WerrCrabReflex)
Biomimetic Omnidirectional Crab Locomotion & Pitchfork Bifurcation Controller
Copyright (c) 2026 ITouch Systems / WERR Robotics.
Licensed under Business Source License 1.1 (BSL 1.1).
"""

from dataclasses import dataclass
import math
import numpy as np

# Canonical Wire Joint Order (22 DoF)
JOINT_NAMES = [
    "LF_J0_joint", "LF_J1_joint", "LF_J2_joint", "LF_J3_joint", "LF_J4_joint", # Front Left Claw (5 DoF)
    "RF_J0_joint", "RF_J1_joint", "RF_J2_joint", "RF_J3_joint", "RF_J4_joint", # Front Right Claw (5 DoF)
    "LM_J0_joint", "LM_J1_joint", "LM_J2_joint",                                 # Left Middle Leg (3 DoF)
    "RM_J0_joint", "RM_J1_joint", "RM_J2_joint",                                 # Right Middle Leg (3 DoF)
    "LR_J0_joint", "LR_J1_joint", "LR_J2_joint",                                 # Left Rear Leg (3 DoF)
    "RR_J0_joint", "RR_J1_joint", "RR_J2_joint",                                 # Right Rear Leg (3 DoF)
]

# Canonical Home Pose for 22 Movable Joints
HOME_POSE = np.array([
    -0.5507, -1.0658, -0.4408, -1.4226,  0.0,     # LF
     0.5524, -1.0660,  0.4414,  1.4212,  0.1,     # RF
     0.0012,  0.5356, -1.8321,                    # LM
    -0.0012, -0.5376,  1.8364,                    # RM
     0.6313,  0.5534, -1.8708,                    # LR
    -0.6326, -0.5548,  1.8739                     # RR
], dtype=np.float64)

@dataclass
class BifurcationState:
    phi: float = 0.0          # Lateral bias state variable
    mu: float = 0.20          # Supercritical parameter (lambda - lambda_c)
    beta: float = 0.22        # Saturation damping parameter
    dt: float = 0.01667       # Control timestep (60 Hz default)

    def step(self, external_bias: float = 0.0) -> float:
        """
        Step pitchfork bifurcation:
        dphi/dt = mu * phi - beta * phi^3 + external_bias
        """
        dphi = self.mu * self.phi - self.beta * (self.phi ** 3) + external_bias
        self.phi += dphi * self.dt
        # Saturation safeguard
        self.phi = float(np.clip(self.phi, -1.5, 1.5))
        return self.phi

class WerrCrabReflex:
    """
    WERR 22-DoF Multi-Quadrant Harmonic Reflex Controller for Jumper Crab Robot.
    """
    def __init__(self, dt: float = 1.0 / 60.0):
        self.dt = dt
        self.time = 0.0
        self.bifurcation = BifurcationState(dt=dt, mu=0.20, beta=0.22)
        self.home_q = HOME_POSE.copy()
        self.current_q = HOME_POSE.copy()
        
        # Quadrant phase offsets for alternating tetrapod gait
        self.phase_q1_lm = 0.0
        self.phase_q2_rm = math.pi
        self.phase_q3_lr = math.pi
        self.phase_q4_rr = 0.0

    def compute_motion(self, t: float, mode: str = "scuttle") -> np.ndarray:
        """
        Synthesize 22-DoF joint angles at time t.
        Modes:
          - 'stand': Neutral equilibrium balance
          - 'scuttle': High-frequency omnidirectional lateral crab crawl
          - 'climb': High-clearance obstacle stepping
          - 'jump_prep': Crouch & spring compression
          - 'jump_air': Airborne dynamic posture stabilization
          - 'victory': Anatolian rhythmic victory claw clapping & bow
        """
        q = self.home_q.copy()
        
        if mode == "stand":
            # Gentle breathing harmonic modulation
            breath = 0.02 * math.sin(2.0 * math.pi * 0.5 * t)
            q[11] += breath  # LM thigh
            q[14] -= breath  # RM thigh
            q[17] += breath  # LR thigh
            q[20] -= breath  # RR thigh
            return q

        elif mode == "scuttle":
            # Lateral crab walk modulated by supercritical pitchfork bifurcation
            freq = 2.4  # Hz
            omega = 2.0 * math.pi * freq
            
            # Step bifurcation with mild sensory noise to break symmetry naturally
            noise = 0.05 * math.sin(0.7 * t)
            phi = self.bifurcation.step(noise)
            direction = math.copysign(1.0, phi if abs(phi) > 0.01 else 1.0)
            
            stride_yaw = 0.28 * direction
            stride_lift = 0.35
            
            # Left Middle (LM)
            q[10] += stride_yaw * math.sin(omega * t + self.phase_q1_lm)
            q[11] += stride_lift * max(0.0, math.cos(omega * t + self.phase_q1_lm))
            q[12] -= 0.5 * stride_lift * max(0.0, math.cos(omega * t + self.phase_q1_lm))
            
            # Right Middle (RM)
            q[13] += stride_yaw * math.sin(omega * t + self.phase_q2_rm)
            q[14] -= stride_lift * max(0.0, math.cos(omega * t + self.phase_q2_rm))
            q[15] += 0.5 * stride_lift * max(0.0, math.cos(omega * t + self.phase_q2_rm))
            
            # Left Rear (LR)
            q[16] += stride_yaw * math.sin(omega * t + self.phase_q3_lr)
            q[17] += stride_lift * max(0.0, math.cos(omega * t + self.phase_q3_lr))
            q[18] -= 0.5 * stride_lift * max(0.0, math.cos(omega * t + self.phase_q3_lr))
            
            # Right Rear (RR)
            q[19] += stride_yaw * math.sin(omega * t + self.phase_q4_rr)
            q[20] -= stride_lift * max(0.0, math.cos(omega * t + self.phase_q4_rr))
            q[21] += 0.5 * stride_lift * max(0.0, math.cos(omega * t + self.phase_q4_rr))
            
            # Counterbalance front claws
            claw_sway = 0.15 * math.sin(omega * t * 0.5)
            q[0] += claw_sway
            q[5] += claw_sway
            q[4] = 0.2 * abs(math.sin(omega * t))  # Left claw subtle grasp
            q[9] = 0.2 * abs(math.sin(omega * t))  # Right claw subtle grasp
            return q

        elif mode == "climb":
            # High-stepping debris clearance gait
            freq = 1.2  # Hz
            omega = 2.0 * math.pi * freq
            lift_high = 0.65
            
            # Stepping cycle
            q[11] += lift_high * max(0.0, math.sin(omega * t))
            q[12] -= 0.8 * lift_high * max(0.0, math.sin(omega * t))
            q[14] -= lift_high * max(0.0, math.sin(omega * t + math.pi))
            q[15] += 0.8 * lift_high * max(0.0, math.sin(omega * t + math.pi))
            
            # Claws in forward probe position
            q[1] += 0.35  # Elevation
            q[6] += 0.35
            return q

        elif mode == "jump_prep":
            # Deep crouch compression
            crouch = 0.55
            q[11] -= crouch
            q[12] += crouch * 1.2
            q[14] += crouch
            q[15] -= crouch * 1.2
            q[17] -= crouch
            q[18] += crouch * 1.2
            q[20] += crouch
            q[21] -= crouch * 1.2
            # Claws tucked
            q[1] -= 0.25
            q[6] -= 0.25
            return q

        elif mode == "jump_air":
            # Full limb extension / aerodynamic flare
            q[11] += 0.70
            q[12] -= 1.10
            q[14] -= 0.70
            q[15] += 1.10
            q[17] += 0.70
            q[18] -= 1.10
            q[20] -= 0.70
            q[21] += 1.10
            # Claws wide open
            q[0] -= 0.40
            q[5] += 0.40
            q[4] = 0.80
            q[9] = 0.80
            return q

        elif mode == "victory":
            # Rhythmic Anatolian claw clapping & celebration ritual
            clap_freq = 3.5  # Hz
            clap = 0.50 * abs(math.sin(2.0 * math.pi * clap_freq * t))
            
            # Lift claws up in proud presentation
            q[0] = -0.15 - 0.30 * math.cos(2.0 * math.pi * 0.8 * t)
            q[1] = -0.30 + clap * 0.4
            q[2] = -0.60
            q[3] = -0.80
            q[4] = clap  # Left pincer clapping
            
            q[5] = 0.15 + 0.30 * math.cos(2.0 * math.pi * 0.8 * t)
            q[6] = -0.30 + clap * 0.4
            q[7] = 0.60
            q[8] = 0.80
            q[9] = clap  # Right pincer clapping
            
            # Body bow & bounce
            bounce = 0.12 * math.sin(2.0 * math.pi * 1.5 * t)
            q[11] += bounce
            q[14] -= bounce
            q[17] += bounce
            q[20] -= bounce
            return q

        return q

def blend_poses(pose_a: np.ndarray, pose_b: np.ndarray, alpha: float) -> np.ndarray:
    """Hermite smoothstep interpolation between two joint configurations."""
    alpha = max(0.0, min(1.0, alpha))
    smooth = alpha * alpha * (3.0 - 2.0 * alpha)
    return (1.0 - smooth) * pose_a + smooth * pose_b

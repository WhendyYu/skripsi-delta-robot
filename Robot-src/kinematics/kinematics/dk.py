"""
Delta Robot Kinematics Core Implementation
"""

import math
from enum import IntEnum
from typing import List, Tuple, Optional
from dataclasses import dataclass


class DeltaStatus(IntEnum):
    """Status codes for delta robot operations"""
    SUCCESS = 0
    UNREACHABLE = 1
    JOINT_LIMIT_EXCEEDED = 2
    SINGULARITY = 3
    IN_COLLISION = 4
    INVALID_INPUT = 5
    COMPUTATION_TIMEOUT = 6


@dataclass
class KinematicsResult:
    """Result container for kinematics operations"""
    success: bool
    status: DeltaStatus
    values: List[float]  # Joint angles for IK (degrees), position for FK
    message: str
    quality: float = 1.0  # Configuration quality metric (0-1)


class DeltaRobot:
    """
    Delta Robot Kinematics Implementation
    Uses radius-based input instead of triangle side length

    Reference: "The Delta Parallel Robot: Kinematics Solutions" by Robert L. Williams II
    """

    def __init__(self, 
                 base_radius: float = 150.0,
                 platform_radius: float = 50.0,
                 upper_arm_length: float = 200.0,
                 forearm_length: float = 300.0,
                 joint_min: float = -60.0,
                 joint_max: float = 60.0):
        """
        Initialize delta robot with geometric parameters

        Args:
            base_radius: Radius of the base triangle circumcircle (mm)
            platform_radius: Radius of the platform triangle circumcircle (mm)
            upper_arm_length: Length of upper arms connected to motors (mm)
            forearm_length: Length of forearms/parallel rods (mm)
            joint_min: Minimum joint angle (degrees)
            joint_max: Maximum joint angle (degrees)
        """
        # Store original parameters
        self.base_radius = base_radius
        self.platform_radius = platform_radius
        self.upper_arm_length = upper_arm_length
        self.forearm_length = forearm_length

        # Joint limits in degrees
        self.joint_min = joint_min
        self.joint_max = joint_max

        # Convert radius to triangle side length for calculations
        # For equilateral triangle: side = radius * sqrt(3)
        self.f = base_radius * math.sqrt(3)  # Base triangle side
        self.e = platform_radius * math.sqrt(3)  # Platform triangle side
        self.rf = upper_arm_length  # Upper arm
        self.re = forearm_length  # Forearm

        # Trigonometric constants
        self.sqrt3 = math.sqrt(3.0)
        self.pi = math.pi
        self.sin120 = self.sqrt3 / 2.0
        self.cos120 = -0.5
        self.tan60 = self.sqrt3
        self.sin30 = 0.5
        self.tan30 = 1.0 / self.sqrt3

        # Safety parameters
        self.singularity_threshold = 0.01  # Minimum determinant threshold
        self.position_tolerance = 1e-6  # mm

        # Pre-compute constant offsets
        self._y1_offset = -0.5 * self.tan30 * self.f  # Base joint y-position
        self._y0_offset = -0.5 * self.tan30 * self.e  # Platform offset
        self._t = (self.f - self.e) * self.tan30 / 2.0  # FK constant

    def inverse_kinematics(self, x: float, y: float, z: float) -> KinematicsResult:
        """
        Inverse Kinematics: Convert end-effector position to joint angles

        Args:
            x, y, z: End effector position in mm (z is typically negative when above base)

        Returns:
            KinematicsResult with joint angles in degrees
        """
        # Input validation
        if not self._validate_input(x, y, z):
            return KinematicsResult(
                success=False,
                status=DeltaStatus.INVALID_INPUT,
                values=[0, 0, 0],
                message="Invalid input: NaN or Inf detected"
            )

        def angle_yz(x0: float, y0: float, z0: float) -> Tuple[DeltaStatus, float]:
            """Calculate angle theta for YZ-plane using sphere-circle intersection"""
            y1 = self._y1_offset
            y0 += self._y0_offset  # shift center to edge

            # Check for singularity (z0 too close to zero)
            if abs(z0) < self.singularity_threshold:
                return DeltaStatus.SINGULARITY, 0.0

            # Linear equation: z = a + b*y
            a = (x0*x0 + y0*y0 + z0*z0 + self.rf*self.rf - self.re*self.re - y1*y1) / (2.0*z0)
            b = (y1 - y0) / z0

            # Discriminant for intersection (must be >= 0 for real solutions)
            d = self.rf*self.rf * (1.0 + b*b) - (a + b*y1)**2
            if d < 0:
                return DeltaStatus.UNREACHABLE, 0.0

            # Calculate joint position (using negative sqrt for elbow-down configuration)
            yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1.0)
            zj = a + b*yj

            # Check for potential singularity in angle calculation
            dy = y1 - yj
            if abs(dy) < self.singularity_threshold:
                return DeltaStatus.SINGULARITY, 0.0

            # Calculate angle (convert to degrees)
            theta = math.degrees(math.atan2(-zj, dy))

            # Ensure angle is in valid range (-180 to 180)
            if theta < -180:
                theta += 360
            elif theta > 180:
                theta -= 360

            return DeltaStatus.SUCCESS, theta

        # Calculate first angle (motor 1 at 0 degrees)
        status, theta1 = angle_yz(x, y, z)
        if status != DeltaStatus.SUCCESS:
            return KinematicsResult(
                success=False,
                status=status,
                values=[0, 0, 0],
                message=self._get_status_message(status)
            )

        # Calculate second angle (motor 2 at +120 degrees)
        status, theta2 = angle_yz(
            x * self.cos120 + y * self.sin120,
            y * self.cos120 - x * self.sin120,
            z
        )
        if status != DeltaStatus.SUCCESS:
            return KinematicsResult(
                success=False,
                status=status,
                values=[0, 0, 0],
                message=self._get_status_message(status)
            )

        # Calculate third angle (motor 3 at -120 degrees)
        status, theta3 = angle_yz(
            x * self.cos120 - y * self.sin120,
            y * self.cos120 + x * self.sin120,
            z
        )
        if status != DeltaStatus.SUCCESS:
            return KinematicsResult(
                success=False,
                status=status,
                values=[0, 0, 0],
                message=self._get_status_message(status)
            )

        # Check joint limits
        angles = [theta1, theta2, theta3]
        for i, angle in enumerate(angles):
            if angle < self.joint_min or angle > self.joint_max:
                return KinematicsResult(
                    success=False,
                    status=DeltaStatus.JOINT_LIMIT_EXCEEDED,
                    values=angles,
                    message=f"Joint {i+1} angle {angle:.1f}° exceeds limits [{self.joint_min}, {self.joint_max}]"
                )

        # Calculate configuration quality
        quality = self._calculate_configuration_quality(angles, x, y, z)

        return KinematicsResult(
            success=True,
            status=DeltaStatus.SUCCESS,
            values=angles,
            message=( f"IK solution found: " f"[{angles[0]:.3f}, " f"{angles[1]:.3f}, " f"{angles[2]:.3f}]" ),
            quality=quality
        )

    def forward_kinematics(self, theta1: float, theta2: float, theta3: float) -> KinematicsResult:
        """
        Forward Kinematics: Convert joint angles to end-effector position

        Args:
            theta1, theta2, theta3: Joint angles in degrees

        Returns:
            KinematicsResult with position [x, y, z] in mm
        """
        # Check joint limits first
        angles = [theta1, theta2, theta3]
        for i, angle in enumerate(angles):
            if angle < self.joint_min or angle > self.joint_max:
                return KinematicsResult(
                    success=False,
                    status=DeltaStatus.JOINT_LIMIT_EXCEEDED,
                    values=[0, 0, 0],
                    message=f"Joint {i+1} angle {angle:.1f}° exceeds limits [{self.joint_min}, {self.joint_max}]"
                )

        # Convert to radians
        theta1_rad = math.radians(theta1)
        theta2_rad = math.radians(theta2)
        theta3_rad = math.radians(theta3)

        # Calculate joint positions (elbow positions)
        y1 = -(self._t + self.rf * math.cos(theta1_rad))
        z1 = -self.rf * math.sin(theta1_rad)

        y2 = (self._t + self.rf * math.cos(theta2_rad)) * self.sin30
        x2 = y2 * self.tan60
        z2 = -self.rf * math.sin(theta2_rad)

        y3 = (self._t + self.rf * math.cos(theta3_rad)) * self.sin30
        x3 = -y3 * self.tan60
        z3 = -self.rf * math.sin(theta3_rad)

        # Check for singularity (denominator too small)
        dnm = (y2 - y1) * x3 - (y3 - y1) * x2
        if abs(dnm) < self.singularity_threshold:
            return KinematicsResult(
                success=False,
                status=DeltaStatus.SINGULARITY,
                values=[0, 0, 0],
                message="Forward kinematics singularity detected"
            )

        # Solve intersection of three spheres
        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3

        # Linear system coefficients
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # Quadratic equation coefficients for z0
        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2.0 * (a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
        c = (b2 - y1*dnm)**2 + b1*b1 + dnm*dnm * (z1*z1 - self.re*self.re)

        # Check discriminant
        discriminant = b*b - 4.0*a*c
        if discriminant < 0.0:
            return KinematicsResult(
                success=False,
                status=DeltaStatus.UNREACHABLE,
                values=[0, 0, 0],
                message="No valid FK solution exists (negative discriminant)"
            )

        # Calculate position (choose lower z solution - above base)
        z0 = -0.5 * (b + math.sqrt(discriminant)) / a
        x0 = (a1*z0 + b1) / dnm
        y0 = (a2*z0 + b2) / dnm

        # Calculate configuration quality
        quality = self._calculate_configuration_quality(angles, x0, y0, z0)

        return KinematicsResult(
            success=True,
            status=DeltaStatus.SUCCESS,
            values=[x0, y0, z0],
            message="FK solution found",
            quality=quality
        )

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """Check if a position is reachable without computing full IK"""
        result = self.inverse_kinematics(x, y, z)
        return result.success

    def get_workspace_bounds(self, z: float) -> Tuple[Optional[float], Optional[float]]:
        """
        Estimate workspace radius at a given Z height
        Returns (min_radius, max_radius) or (None, None) if not reachable
        """
        # Binary search for maximum reachable radius at given Z
        max_radius = 0.0
        for r in [i * 10.0 for i in range(50)]:  # Test up to 500mm
            if not self.is_reachable(r, 0, z) and not self.is_reachable(0, r, z):
                break
            max_radius = r

        if max_radius == 0.0:
            return None, None

        return 0.0, max_radius

    def _calculate_configuration_quality(self, angles: List[float], x: float, y: float, z: float) -> float:
        """
        Calculate configuration quality metric (0-1)
        Higher quality means:
        - Further from joint limits
        - Further from singularities
        - More centered in workspace
        """
        # Joint limit proximity (0 = at limit, 1 = center of range)
        joint_range = self.joint_max - self.joint_min
        joint_center = (self.joint_max + self.joint_min) / 2.0

        joint_quality = 1.0
        for angle in angles:
            # Distance from nearest limit
            dist_from_min = abs(angle - self.joint_min)
            dist_from_max = abs(angle - self.joint_max)
            dist_from_center = abs(angle - joint_center)

            # Quality decreases as we approach limits
            limit_proximity = min(dist_from_min, dist_from_max) / (joint_range / 2.0)
            joint_quality = min(joint_quality, limit_proximity)

        # Singularity proximity (based on Z position)
        # Singularities occur when z approaches 0 (platform at base height)
        z_proximity = min(abs(z) / 50.0, 1.0)  # Normalize, cap at 1

        # Combine qualities (weighted average)
        quality = 0.6 * joint_quality + 0.4 * z_proximity

        return max(0.0, min(1.0, quality))  # Clamp to [0, 1]

    def _validate_input(self, x: float, y: float, z: float) -> bool:
        """Validate input values are finite numbers"""
        return all(math.isfinite(v) for v in [x, y, z])

    def _get_status_message(self, status: DeltaStatus) -> str:
        """Get human-readable status message"""
        messages = {
            DeltaStatus.SUCCESS: "Operation successful",
            DeltaStatus.UNREACHABLE: "Position is outside workspace",
            DeltaStatus.JOINT_LIMIT_EXCEEDED: "Joint angle limits exceeded",
            DeltaStatus.SINGULARITY: "Robot singularity detected",
            DeltaStatus.IN_COLLISION: "Position would cause collision",
            DeltaStatus.INVALID_INPUT: "Invalid input parameters",
            DeltaStatus.COMPUTATION_TIMEOUT: "Computation timeout"
        }
        return messages.get(status, "Unknown error")


# Convenience functions for quick testing
def create_standard_robot() -> DeltaRobot:
    """Create a delta robot with standard dimensions"""
    return DeltaRobot(
        base_radius=200.0,
        platform_radius=50.0,
        upper_arm_length=100.0,
        forearm_length=300.0,
        joint_min=-90.0,
        joint_max=90.0
    )


if __name__ == "__main__":
    # Simple test
    robot = create_standard_robot()

    # Test IK
    result = robot.inverse_kinematics(0, 0, -200)
    print(f"IK test: {result}")

    # Test FK
    result = robot.forward_kinematics(0, 0, 0)
    print(f"FK test: {result}")

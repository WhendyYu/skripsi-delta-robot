"""
Delta Robot Inverse Kinematics Service Node
Provides batch IK solving capability via ROS2 service
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from msgs.srv import SolveIK

from .dk import DeltaRobot


class IKServiceNode(Node):
    """ROS2 Service node for delta robot inverse kinematics"""
    
    def __init__(self):
        super().__init__('ik_service')
        
        # Declare parameters with defaults (in degrees for config)
        self.declare_parameter('base_radius', 200.0)  # mm
        self.declare_parameter('platform_radius', 50.0)  # mm
        self.declare_parameter('upper_arm_length', 100.0)  # mm
        self.declare_parameter('forearm_length', 300.0)  # mm
        self.declare_parameter('joint_min', -90.0)  # degrees
        self.declare_parameter('joint_max', 90.0)  # degrees
        
        # Get parameters
        self._load_parameters()
        
        # Create delta robot instance
        self._create_robot()
        
        # Create service
        self.srv = self.create_service(
            SolveIK,
            'solve_ik',
            self.solve_ik_callback
        )
        
        # Set up parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info(
            f'IK Service started with robot parameters:\n'
            f'  Base radius: {self.base_radius} mm\n'
            f'  Platform radius: {self.platform_radius} mm\n'
            f'  Upper arm: {self.upper_arm_length} mm\n'
            f'  Forearm: {self.forearm_length} mm\n'
            f'  Joint limits: [{self.joint_min}°, {self.joint_max}°]'
        )
        
    def _load_parameters(self):
        """Load parameters from parameter server"""
        self.base_radius = self.get_parameter('base_radius').value
        self.platform_radius = self.get_parameter('platform_radius').value
        self.upper_arm_length = self.get_parameter('upper_arm_length').value
        self.forearm_length = self.get_parameter('forearm_length').value
        self.joint_min = self.get_parameter('joint_min').value
        self.joint_max = self.get_parameter('joint_max').value
        
    def _create_robot(self):
        """Create or recreate the delta robot instance with current parameters"""
        self.robot = DeltaRobot(
            base_radius=self.base_radius,
            platform_radius=self.platform_radius,
            upper_arm_length=self.upper_arm_length,
            forearm_length=self.forearm_length,
            joint_min=self.joint_min,
            joint_max=self.joint_max
        )
        
    def parameters_callback(self, params):
        """Handle dynamic parameter updates"""
        for param in params:
            if param.name in ['base_radius', 'platform_radius', 
                             'upper_arm_length', 'forearm_length',
                             'joint_min', 'joint_max']:
                self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        
        # Reload parameters and recreate robot
        self._load_parameters()
        self._create_robot()
        
        return SetParametersResult(successful=True)
    
    def solve_ik_callback(self, request, response):
        """
        Service callback for batch IK solving
        
        Args:
            request: SolveIK request with positions array
            response: SolveIK response with joint_angles and reachable arrays
        
        Returns:
            response with success=True only if ALL positions are reachable
        """
        try:
            num_positions = len(request.positions)
            
            if num_positions == 0:
                self.get_logger().warning('Received empty positions array')
                response.success = False
                response.joint_angles = []
                response.reachable = []
                return response
            
            # Pre-allocate arrays
            joint_angles = []
            reachable = []
            all_reachable = True
            
            # Process each position
            for i, point in enumerate(request.positions):
                x_mm = point.x
                y_mm = point.y
                z_mm = point.z
                
                # Solve IK
                ik_result = self.robot.inverse_kinematics(x_mm, y_mm, z_mm)
                
                if ik_result.success:
                    # Append joint angles (in degrees as per interface requirement)
    
                    joint_angles.extend(ik_result.values)  # [theta1, theta2, theta3]
                    reachable.append(True)
                    
                    self.get_logger().debug(
                        f'Position {i}: ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm -> '
                        f'θ1={ik_result.values[0]:.2f}°, '
                        f'θ2={ik_result.values[1]:.2f}°, '
                        f'θ3={ik_result.values[2]:.2f}°'
                    )
                else:
                    # Position not reachable - append zeros for joint angles
                    joint_angles.extend([0.0, 0.0, 0.0])
                    reachable.append(False)
                    all_reachable = False
                    
                    self.get_logger().warning(
                        f'Position {i}: ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm '
                        f'is not reachable: {ik_result.message}'
                    )
            
            # Fill response
            response.success = all_reachable  # True only if ALL positions are reachable
            response.joint_angles = joint_angles  # Flattened array in degrees
            response.reachable = reachable
            
            # Log summary
            reachable_count = sum(reachable)
            self.get_logger().info(
                f'IK batch solved: {reachable_count}/{num_positions} positions reachable. '
                f'Overall success: {all_reachable}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in IK service: {str(e)}')
            response.success = False
            response.joint_angles = []
            response.reachable = []
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IKServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
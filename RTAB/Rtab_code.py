import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_node')

        self.rtabmap_subscriber = self.create_subscription(Odometry, '/rtabmap/odom', self.rtabmap_callback, 10)
        self.terrasentia_subscriber = self.create_subscription(Odometry, '/terrasentia/ekf', self.terrasentia_callback, 10)

        self.rtabmap_file = open('rtab_traj.txt', 'w')
        self.terrasentia_file = open('ekf_traj.txt', 'w')

        self.timer1 = self.create_timer(1.0, self.timer_callback)
        self.timer2 = self.create_timer(1.0, self.timer_callback)

        self.est_data = None
        self.gt_data = None

    def rtabmap_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.rtabmap_file.write(f'{x} {y} {z}\n')

    def terrasentia_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.terrasentia_file.write(f'{x} {y} {z}\n')

    def timer_callback(self):
        self.current_time = self.get_clock().now().nanoseconds / 1e9
        self.last_time = self.current_time

    def plot_trajectory(self):
        
        estimated_x, estimated_y, estimated_z = self.est_data[:, 0], self.est_data[:, 1], self.est_data[:, 2]
        ground_truth_x, ground_truth_y, ground_truth_z = self.gt_data[:, 0], self.gt_data[:, 1], self.gt_data[:, 2]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(estimated_x, estimated_y, estimated_z, label='RTAB', color='red')
        ax.plot(ground_truth_x, ground_truth_y, ground_truth_z, label='EKF', color='green')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()

        x = np.sqrt(np.mean((estimated_x - ground_truth_x)**2))
        y = np.sqrt(np.mean((estimated_y - ground_truth_y)**2))
        z = np.sqrt(np.mean((estimated_z - ground_truth_z)**2))

        rmse_val = np.sqrt(np.mean((self.est_data - self.gt_data)**2, axis=0))

        print(f"X: {x}")
        print(f"Y: {y}")
        print(f"Z: {z}")
        print(f"rmse: {rmse_val}")

def main(args=None):
    rclpy.init(args=args)

    trajectory_node = TrajectoryNode()

    try:
        rclpy.spin(trajectory_node)
    except KeyboardInterrupt:
        pass

    trajectory_node.est_data = np.loadtxt('rtab_traj.txt')
    trajectory_node.gt_data = np.loadtxt('ekf_traj.txt')

    trajectory_node.plot_trajectory()

    trajectory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

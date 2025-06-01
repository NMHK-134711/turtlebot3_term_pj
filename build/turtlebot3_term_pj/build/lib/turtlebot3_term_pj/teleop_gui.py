import tkinter as tk
import rclpy
from geometry_msgs.msg import Twist

class TeleopGUI:
    def __init__(self, node):
        self.node = node
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        self.root = tk.Tk()
        self.root.title("TurtleBot Teleop")

        self.forward_button = tk.Button(self.root, text="Forward", command=self.move_forward)
        self.forward_button.pack()

        self.backward_button = tk.Button(self.root, text="Backward", command=self.move_backward)
        self.backward_button.pack()

        self.left_button = tk.Button(self.root, text="Left", command=self.turn_left)
        self.left_button.pack()

        self.right_button = tk.Button(self.root, text="Right", command=self.turn_right)
        self.right_button.pack()

        self.stop_button = tk.Button(self.root, text="Stop", command=self.stop)
        self.stop_button.pack()

    def move_forward(self):
        self.twist.linear.x = 0.2
        self.pub.publish(self.twist)

    def move_backward(self):
        self.twist.linear.x = -0.2
        self.pub.publish(self.twist)

    def turn_left(self):
        self.twist.angular.z = 0.5
        self.pub.publish(self.twist)

    def turn_right(self):
        self.twist.angular.z = -0.5
        self.pub.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)

def main():
    rclpy.init()
    node = rclpy.create_node('teleop_gui')
    gui = TeleopGUI(node)
    gui.root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
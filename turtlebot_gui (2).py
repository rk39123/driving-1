import sys
import queue
import rclpy
import threading
import signal
from functools import partial

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action.client import GoalStatus


class NODE(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.emit_signal = None
        
        #init
        self.num = 3
        self.init_pose = [-2.0, -0.5, 0.0, 1.0]
        self.goal_poses = [[0.0,0.0] for i in range(self.num)]
        self.setting_poses = [False for i in range(self.num)]

        # Queue
        self.queue = queue.Queue()
        self.timer = self.create_timer(0.1, self.process_queue)

        # create
        self.clicked_point_subscriber = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10
            )
        
        self.set_initial_pose_service_client = self.create_client(
            SetInitialPose, 
            '/set_initial_pose'
            )
        
        self.navigate_to_pose_action_client = ActionClient(
            self, 
            NavigateToPose, 
            "navigate_to_pose")
        
        # Init function
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')
        
    
    def process_queue(self):
        while not self.queue.empty():
            cmd, val = self.queue.get()
            if cmd == 'subscriber_clicked_point':
                self.setting_poses[val] = True

            elif cmd == 'navigate_to_pose_send_goal':
                self.navigate_to_pose_send_goal(val)

            elif cmd == 'set_initial_pose':
                self.set_initial_pose()



    # Topic subscriber GET POSE
    def clicked_point_callback(self, msg):
        self.get_logger().info(f'Received point: x={msg.point.x}, y={msg.point.y}')
        for i in range(self.num):
            if self.setting_poses[i]:
                x = round(float(msg.point.x),1)
                y = round(float(msg.point.y),1)   
                self.goal_poses[i][0] = x
                self.goal_poses[i][1] = y

                text = f"x= {x:.1f}, y= {y:.1f}"
                self.emit_signal(['set_label', i, text])

                message = f"[GET] table_{i} is x= {x:.1f}, y= {y:.1f}"
                self.emit_signal(['append_textBrowser', message])
                self.setting_poses[i] = False

    # Service client SET INIT POSE ESTIMATE
    def set_initial_pose(self):
        x, y, z, w = self.init_pose
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        future = self.set_initial_pose_service_client.call_async(req)
        
        if future.result() is not None:
            message = "[INFO] Initial pose set successfully"
        else:
            message = "[WARN] Failed to set initial pose"
        
        self.emit_signal(['append_textBrowser', message])
        
        return future.result()

    ## Action client NAVIGATE      
    def navigate_to_pose_send_goal(self, i):
        wait_count = 1
        while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                message = "[WARN] Navigate action server is not available."
                self.emit_signal(['append_textBrowser', message])
                return False
            wait_count += 1
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.goal_poses[i][0]
        goal_msg.pose.pose.position.y = self.goal_poses[i][1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
        
        return True

    def navigate_to_pose_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = "[WARN] Action goal rejected."
            self.emit_signal(['append_textBrowser', message])
            return

        message = "[INFO] Action goal accepted."
        self.emit_signal(['append_textBrowser', message])
        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)

    def navigate_to_pose_action_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback
        # self.get_logger().info("Action feedback: {0}".format(action_feedback))

    def navigate_to_pose_action_result(self, future):
        action_status = future.result().status
        action_result = future.result().result
        if action_status == GoalStatus.STATUS_SUCCEEDED:

            message = "[INFO] Action succeeded!."
            self.emit_signal(['append_textBrowser', message])

        else:
            message = f"[WARN] Action failed with status: {action_status}"
            self.emit_signal(['append_textBrowser', message])

    def set_emit_signal(self, emit_func):
        self.emit_signal = emit_func


class GUI(QMainWindow):
    received_signal = Signal(list)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.num = self.node.num

        self.received_signal.connect(self.process_signal)
        self.setupUi()
        self.initialize_signal()
        self.set_initial_pose()

    def setupUi(self):
        
        self.setWindowTitle(u"MainWindow")
        self.resize(274, 180 + 40 *self.num)

        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName(u"centralwidget")
        
        self.getButtons = [None for i in range(self.num)]
        self.getButtons_geo = [19, 20, 51, 23]
        for i in range(self.num):
            self.getButtons[i] = QPushButton(self.centralwidget)
            self.getButtons[i].setObjectName(f"getButton_{i}")
            self.getButtons[i].setGeometry(QRect(*self.getButtons_geo))
            self.getButtons[i].setText(f"get{i}")
            self.getButtons[i].clicked.connect(partial(self.get_button_clicked, i))
            self.getButtons_geo[1] +=40
            
        self.labels = [None for i in range(self.num)]
        self.labels_geo = [90, 20, 131, 23]
        for i in range(self.num):
            self.labels[i] = QLabel(self.centralwidget)
            self.labels[i].setObjectName(f"label_{i}")
            self.labels[i].setGeometry(QRect(*self.labels_geo))
            self.labels[i].setText("x= 0.0, y= 0.0")
            self.labels_geo[1] +=40
        
        self.goButtons_geo = [200, 20, 51, 23]
        self.goButtons = [None for i in range(self.num)]
        for i in range(self.num):
            self.goButtons[i] = QPushButton(self.centralwidget)
            self.goButtons[i].setObjectName(f"goButton_{i}")
            self.goButtons[i].setGeometry(QRect(*self.goButtons_geo))
            self.goButtons[i].setText(f"go{i}")
            self.goButtons[i].clicked.connect(partial(self.go_button_clicked, i))
            self.goButtons_geo[1] +=40
        
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setObjectName(u"textBrowser")
        self.textBrowser.setGeometry(QRect(20, 20 +40 *self.num, 231, 141))

        self.setCentralWidget(self.centralwidget)

    def initialize_signal(self):
        self.node.set_emit_signal(self.received_signal.emit)
    
    def process_signal(self, signal):
        cmd = signal[0]
        if cmd == 'append_textBrowser':
            text = signal[1]
            self.textBrowser.append(text)
        elif cmd == 'set_label':
            i, text = signal[1:]
            self.labels[int(i)].setText(text)

    def get_button_clicked(self, i):
        self.node.queue.put(['subscriber_clicked_point' ,i])

    def go_button_clicked(self, i):
        self.node.queue.put(['navigate_to_pose_send_goal' ,i])

    def set_initial_pose(self):
        self.node.queue.put(['set_initial_pose' ,0])

def main():
    rclpy.init()
    node = NODE()

    app = QApplication(sys.argv)
    gui = GUI(node)

    ros_thread = threading.Thread(target=lambda : rclpy.spin(node), daemon=True)
    ros_thread.start()

    gui.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        sys.exit(0)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
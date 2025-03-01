import sys
import rclpy
import threading
import signal
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtWidgets import *

class NODE(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.emit_signal = None

        self.subscription = self.create_subscription(
                String, 'message', self.subscription_callback, 10)

    def subscription_callback(self, msg):
        message = msg.data
        self.get_logger().info(f'Received message: {message}')

        if self.emit_signal is not None:
            self.emit_signal(message)
        else:
            self.get_logger().info(f'Node-Gui no connected')

    def set_emit_signal(self, emit_func):
        self.emit_signal = emit_func


class GUI(QMainWindow):
    message_received = Signal(str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.message_received.connect(self.add_message)
        self.setupUi()
        self.initialize_signal()

    def setupUi(self):
        if not self.objectName():
            self.setObjectName(u"MainWindow")

        self.setObjectName("MainWindow")
        self.resize(375, 310)
        
        self.centralwidget = QWidget(self)
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QRect(60, 90, 256, 192))
        
        self.setCentralWidget(self.centralwidget)

    def add_message(self, message):
        self.textBrowser.append(message)

    def initialize_signal(self):
        self.node.set_emit_signal(self.message_received.emit)
        


def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node))
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
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
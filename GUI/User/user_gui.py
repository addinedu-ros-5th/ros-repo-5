import sys
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.uic import loadUi
#from PyQt5.QtWidgets import QWidget

import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
import uuid

from datetime import datetime

from_class = uic.loadUiType("GUI/User/user_gui.ui")[0]   #change path

class WindowClass(QMainWindow, from_class) :
    def __init__(self, ros2_thread):
        super().__init__()
        self.setupUi(self)

        #===design===
        self.setWindowTitle("Hello, Qt!")
        self.setFixedSize(750, 550)
        button_list = [self.pbt_num1, self.pbt_num2, self.pbt_num3, self.pbt_num4, self.pbt_num5,
                       self.pbt_num6, self.pbt_num7, self.pbt_num8, self.pbt_num9, self.pbt_undo]
        for button in button_list:   #button size setting
            button.setFixedSize(95, 95)
        self.pbt_num0.setFixedSize(197, 95)


        #===setting===
        self.ros2_thread = ros2_thread
        self.ros2_thread.rx_signal.connect(self.receive_ros2_data)   #if receive data from ros2

        self.car_num_data = []   #full car_num
        self.car_num_text = ""   #4 car num
        self.selected_num = ""   #선택된 차량, lb_carnum에 들어갈 번호가 저장되는 변수
        self.car_waiting_list = []   #출차 예정 차량 리스트
        self.history_waiting_list = []   #출차 예정 차량 리스트 기록
        self.car_ready_list = []   #출차 대기 차량 리스트(출차 예정 -> 출차 대기)

        #===message data===
        self.req_ready = "req_ready"
        self.req_out = "req_out"
        self.req_cancle = "req_cancle"


        #===message data end===
        self.timer_stk01 = QTimer(self)   #move page timer_stk01
        self.timer_stk02 = QTimer(self)   #move page timer_stk02

        self.user_to_server_pub = User_To_Server_Pub()
        

        #===stack0:page_input_num===
        self.pbt_num0.clicked.connect(lambda: self.input_num_from_pbt(0))   #input num
        self.pbt_num1.clicked.connect(lambda: self.input_num_from_pbt(1))
        self.pbt_num2.clicked.connect(lambda: self.input_num_from_pbt(2))
        self.pbt_num3.clicked.connect(lambda: self.input_num_from_pbt(3))
        self.pbt_num4.clicked.connect(lambda: self.input_num_from_pbt(4))
        self.pbt_num5.clicked.connect(lambda: self.input_num_from_pbt(5))
        self.pbt_num6.clicked.connect(lambda: self.input_num_from_pbt(6))
        self.pbt_num7.clicked.connect(lambda: self.input_num_from_pbt(7))
        self.pbt_num8.clicked.connect(lambda: self.input_num_from_pbt(8))
        self.pbt_num9.clicked.connect(lambda: self.input_num_from_pbt(9))
        self.pbt_undo.clicked.connect(self.remove_num)   #remove num
        
        self.timer_stk01.timeout.connect(self.move_page_check)   #move page timer
        self.timer_stk02.timeout.connect(self.move_page_select)

        self.table_waiting.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)   #table column size setting
        self.table_waiting.setEditTriggers(QAbstractItemView.NoEditTriggers)   #fix data
        self.table_ready.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_ready.setEditTriggers(QAbstractItemView.NoEditTriggers)

        #===stack1:page_check===
        self.pbt_check_cancle.clicked.connect(lambda: self.back_page_input_num(0))   #move page
        self.pbt_out_signal.clicked.connect(lambda: self.back_page_input_num(1))

        self.in_time_info = ""

        
        #===stack2:page_select===
        self.table_select_car.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_select_car.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table_select_car.cellClicked.connect(self.select_to_check)
        self.table_ready.cellClicked.connect(self.send_car_out)
        self.pbt_select_cancle.clicked.connect(lambda: self.back_page_input_num(2))
        

    #===stk0 function===
    def input_num_from_pbt(self, num):   #input num
        if len(self.car_num_text) < 4:
            self.car_num_text += str(num)
        self.lb_input_carnum.setText(self.car_num_text)

        if len(self.car_num_text) == 4:
            send_text = "D" + self.car_num_text
            self.user_to_server_pub.user_to_server_pub_msg(send_text)   #gui:4 car num -> server

    def remove_num(self):   #remove num
        self.car_num_text = self.car_num_text[:-1]
        
        if self.car_num_text == "":
            #print("not data")
            self.lb_input_carnum.clear()
        else:
            #print(self.car_num_text)
            self.lb_input_carnum.setText(self.car_num_text)


    def send_car_out(self, row, column):   #G1 or F1 -> O1:send req_out Signal
        self.selected_num = self.table_ready.item(row, column).text()
        send_text = "S" + self.req_out + "N" + self.selected_num
        self.user_to_server_pub.user_to_server_pub_msg(send_text)

        self.car_ready_list = [line for line in self.car_ready_list if self.selected_num not in line]
        self.table_ready.setRowCount(len(self.car_ready_list))
        self.table_ready.setColumnCount(1)
        for row_idx, row_data in enumerate(self.car_ready_list):
            self.table_ready.setItem(row_idx, 0, QTableWidgetItem(row_data))
        

    def move_page_check(self):
        self.timer_stk01.stop()
        self.stk_user.setCurrentIndex(1)
        self.lb_carnum.setText(self.selected_num)
        self.clear_input_carnum()
            
    def move_page_select(self):
        self.timer_stk02.stop()
        self.stk_user.setCurrentIndex(2)
        self.lb_carnum.setText(self.selected_num)
        self.clear_input_carnum()
        
    def clear_input_carnum(self):
        self.lb_input_carnum.clear()
        self.car_num_text = ""


    def back_page_input_num(self, status):   #cancle button and req_ready
        if status == 0:   #stk1 -> 0:page_check cancle
            send_text = "S" + self.req_cancle
            self.user_to_server_pub.user_to_server_pub_msg(send_text)   #ros2:send:cancle signal1->0
            self.stk_user.setCurrentIndex(0)
            self.selected_num = ""
            self.lb_carnum.clear()
            self.car_num_data = []
            #print("Reset")

        elif status == 1:   #stk1 -> 0:send out signal
            reply = QMessageBox.question(self, "check again", "정말 출차하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                send_text = "S" + self.req_ready
                self.user_to_server_pub.user_to_server_pub_msg(send_text)   #ros2:send:out_signal
                self.stk_user.setCurrentIndex(0)
                if self.selected_num not in self.history_waiting_list:
                    self.car_waiting_list.append(self.selected_num)
                    self.history_waiting_list.append(self.selected_num)
                else:
                    #print("이미 요청된 차량")
                    QMessageBox.information(self, "info", "이미 요청된 차량입니다.")

                self.table_waiting.setRowCount(len(self.car_waiting_list))
                self.table_waiting.setColumnCount(1)
                for row_idx, row_data in enumerate(self.car_waiting_list):
                    self.table_waiting.setItem(row_idx, 0, QTableWidgetItem(row_data))

                self.selected_num = ""
                self.lb_carnum.clear()
                #print("Signal")
                self.car_num_data = []
            else:
                return
            
        elif status == 2:   #stk2 -> 0:page_select cancle
            send_text = "S" + self.req_cancle
            self.user_to_server_pub.user_to_server_pub_msg(send_text)   #ros2:send:cancle signal2->0
            self.stk_user.setCurrentIndex(0)
            self.car_num_data = []
            #print("back select")
            

    #===stk2 function===
    def select_to_check(self, row, column):
        self.table_select_car.clearSelection()
        self.table_select_car.setFocus()
        self.selected_num = self.table_select_car.item(row, column).text()
        #print(f"ROW {row} clicked: {self.selected_num}")
        send_text = "D" + self.selected_num
        self.user_to_server_pub.user_to_server_pub_msg(send_text)
        self.stk_user.setCurrentIndex(1)
        self.lb_carnum.setText(self.selected_num)

    #===stkxxx===
    def receive_ros2_data(self, msg):   #server:full car num -> gui
        print("data check: ", msg)
        
        if msg[0] == "D" and msg[-1] != "R":
            receive_data = msg[1:]
            find_t = receive_data.find("T")
            if find_t == -1:   #no T   #2 more
                self.car_num_data.append(receive_data)


                if len(self.car_num_data) >= 2:
                    #print("select: ", self.car_num_data)
                    self.timer_stk02.start(500)
                    self.table_select_car.setRowCount(len(self.car_num_data))
                    self.table_select_car.setColumnCount(1)
                    for row_idx, num_data in enumerate(self.car_num_data):
                        self.table_select_car.setItem(row_idx, 0, QTableWidgetItem(num_data))
            else:   #T one
                self.car_num_data = []
                receive_car_num = receive_data[:find_t]
                self.in_time_info = receive_data[find_t+1:]
                #print(receive_car_num, " + " , self.in_time_info)

                self.car_num_data.append(receive_car_num)

                self.timer_stk01.start(500)
                self.selected_num = self.car_num_data[0]
                #print("select: ", self.selected_num)
                self.lb_in_time.setText(self.in_time_info)

                cur_time = datetime.now().strftime("%m/%d %H:%M")
                self.lb_out_time.setText(cur_time)

                if len(self.car_waiting_list) == 0:
                    spend_time = 2
                else:
                    spend_time = len(self.car_waiting_list) * 3

                self.lb_spend.setText(f"약 {spend_time}분 정도가 소요됩니다.")


        elif msg[0] == "D" and msg[-1] == "R":   #table_waiting -> table_ready
            ready_com_data = msg[1:-1]
            self.car_waiting_list = [line for line in self.car_waiting_list if ready_com_data not in line]
            self.car_ready_list.append(ready_com_data)
            self.table_ready.setRowCount(len(self.car_ready_list))
            self.table_ready.setColumnCount(1)
            for row_idx, row_data in enumerate(self.car_ready_list):
                self.table_ready.setItem(row_idx, 0, QTableWidgetItem(row_data))

            self.table_waiting.setRowCount(len(self.car_waiting_list))
            self.table_waiting.setColumnCount(1)
            for row_idx, row_data in enumerate(self.car_waiting_list):
                self.table_waiting.setItem(row_idx, 0, QTableWidgetItem(row_data))

    #===stkxx end===

class User_To_Server_Pub(Node):
    def __init__(self):
        unique_node_name = f"user_to_server_{uuid.uuid4().hex}"
        super().__init__(unique_node_name)
        self.publisher = self.create_publisher(String, "/user_to_server", 10)

    def user_to_server_pub_msg(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

class Server_To_User_Sub(Node):
    def __init__(self):
        unique_node_name = f"server_to_user_{uuid.uuid4().hex}"
        super().__init__(unique_node_name)
        self.subscription = self.create_subscription(
            String,
            "/server_to_user",
            self.listener_callback,
            10
        )
        self.subscription
        self.latest_msg = ""

    def listener_callback(self, msg):
        self.latest_msg = msg.data
        self.get_logger().info(f"Received:'{msg.data}'")

class Ros2Thread(QThread):
    rx_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        rp.init()
        self.node = Server_To_User_Sub()

    def run(self):
        while rp.ok():
            rp.spin_once(self.node)
            if self.node.latest_msg:
                self.rx_signal.emit(self.node.latest_msg)
    
#def main(args=None):       
def main():

    #rp.init(args=args)

    app = QApplication(sys.argv)

    ros2_thread = Ros2Thread()
    #myWindows = WindowClass()
    myWindows = WindowClass(ros2_thread)
    myWindows.show()
    ros2_thread.start()

    try:
        sys.exit(app.exec_())
    finally:
        rp.shutdown()
        ros2_thread.node.destroy_node()


if __name__ == "__main__":
    main()

    
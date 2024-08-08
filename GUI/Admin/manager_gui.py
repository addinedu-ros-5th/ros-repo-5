import sys
from PyQt5.QtCore import Qt, QPoint, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QMouseEvent
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.uic import loadUi
from DB_set import DB_SET
from park_geo import park_geo
import mysql.connector
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np

import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

#from PyQt5.QtWidgets import QWidget

#다음 목표:그래프 그릴 수 있도록 틀 생성
#       :타이머 -> 신호를 통해 맵 및 정보 업데이트

#===setting===
class BDConnector:
    def __init__(self):
        self.connection = None
        self.cursor = None
    
    def connect_to_database(self):
        global connection, cursor
        try:
            connection = mysql.connector.connect(
                host=DB_SET['HOST'],
                database=DB_SET['DATABASE'],
                user=DB_SET['USER'],
                password=DB_SET['PASSWORD']
            )

            if connection.is_connected():
                cursor = connection.cursor()
                return connection

        except mysql.connector.Error as e:
            print(f"Error: {e}")


from_class = uic.loadUiType("GUI/Admin/manager_gui.ui")[0]   #change path
#===main window===
class WindowClass(QMainWindow, BDConnector, from_class) :
    def __init__(self, ros2_thread):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("Hello, Qt!")
        self.setFixedSize(750, 550)

        self.ros2_thread = ros2_thread
        self.ros2_thread.rx_signal.connect(self.received_and_print)

        self.timer_table_map_info = QTimer(self)
        self.timer_table_map_info.timeout.connect(self.load_map_info)
        self.timer_table_map_info.start(1000)
        self.check_count = 0

        self.table_check_flow.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_check_flow.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table_map_info.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_map_info.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self.pbt_exit.clicked.connect(self.exit_window)

        self.pbt_A.clicked.connect(self.open_total_graph)
        self.pbt_B.clicked.connect(self.open_map_info)
        self.pbt_D.clicked.connect(self.check_value)

        self.pbt_C.hide()
        self.pbt_D.hide()

        self.le_ID.returnPressed.connect(self.check_manager_info)
        self.le_PW.returnPressed.connect(self.check_manager_info)

        self.ctrl_e = QShortcut(QKeySequence("Ctrl+E"), self)
        self.ctrl_e.activated.connect(self.back_to_login)

        self.page_login.setFocus()
        self.opened_window = []

        self.flag_open_graph = False
        self.flag_open_map = False

        self.load_map_info()

        self.test_list = []

        self.line_R1 = "R1, wait, wait, wait"
        self.line_R2 = "R2, wait, wait, wait"
        self.line_R3 = "R3, wait, wait, wait"

        self.table_data_list = []

        self.table_set_base = " ,  ,  ,  "
        self.table_data_list.append(self.line_R1)
        self.table_data_list.append(self.line_R2)
        self.table_data_list.append(self.line_R3)

        self.table_show()

    #===test===
    def table_show(self):
        num_rows = len(self.table_data_list)
        num_columns = len(self.table_data_list[0].split(', '))

        self.table_check_flow.setRowCount(num_rows)
        self.table_check_flow.setColumnCount(num_columns)

        for i, item in enumerate(self.table_data_list):
            data = item.split(', ')
            for j, value in enumerate(data):
                self.table_check_flow.setItem(i, j, QTableWidgetItem(value))

    def modify_table_data(self, row, column, new_value):
        data = self.table_data_list[row].split(', ')
        data[column] = new_value
        self.table_data_list[row] = ', '.join(data)

    def received_and_print(self, msg):
        received_data = msg   #BR1B030 or PR1SI2TB2

        if received_data[0] == "B":   #battery
            step1 = received_data[1:]
            self.get_robot_battery(step1)

        elif received_data[0] == "P":   #position
            step1 = received_data[1:]
            self.get_robot_position(step1)

        print("data check: ", msg)


    def get_robot_battery(self, data):
        data = data
        rb_id = data[0:2]
        battery = data[3:]

        if battery[0] == "0" and battery[1] == "0":  #0~9
            rb_battery = battery[2:]
        elif battery[0] == "0" and battery[1] != "0":   #10~99
            rb_battery = battery[1:]
        else:  #100
            rb_battery = battery

        if rb_id == "R1":
            self.modify_table_data(0, 1, rb_battery)

        elif rb_id == "R2":
            self.modify_table_data(1, 1, rb_battery)

        elif rb_id == "R3":
            self.modify_table_data(2, 1, rb_battery)

        else:   #pass
            pass

        self.table_show()



    def get_robot_position(self, data):
        data = data
        rb_id = data[0:2]
        rb_stpt = data[3:5]
        rb_ept = data[6:8]

        if rb_id == "R1":
            self.modify_table_data(0, 2, rb_stpt)
            self.modify_table_data(0, 3, rb_ept)

        elif rb_id == "R2":
            self.modify_table_data(1, 2, rb_stpt)
            self.modify_table_data(1, 3, rb_ept)

        elif rb_id == "R3":
            self.modify_table_data(2, 2, rb_stpt)
            self.modify_table_data(2, 3, rb_ept)

        else:   #pass
            pass

        self.table_show()

    #===open window function===
    def open_total_graph(self):
        if self.flag_open_graph == False:
            self.graph_window = TotalGraph(self)
            self.graph_window.move(self.pos())
            self.graph_window.show()
            self.opened_window.append(self.graph_window)
            self.flag_open_graph = True
        else:
            pass

    def open_map_info(self):
        if self.flag_open_map == False:
            self.map_window = MapInfo(self)
            self.map_window.move(self.pos())
            self.map_window.show()
            self.opened_window.append(self.map_window)
            self.flag_open_map = True
        else:
            pass


    #======
    def check_manager_info(self):
        box_id = self.le_ID.text()
        box_pw = self.le_PW.text()

        try:
            self.connect_to_database()

            query = "SELECT ID, PW FROM IDPW WHERE ID=%s AND PW=%s"
            cursor.execute(query, (box_id, box_pw))
            found = cursor.fetchone()

            if found:
                print("OK")
                self.stk_manager.setCurrentIndex(1)
            else:
                print("Not Found")

        except mysql.connector.Error as e:
            print("Error")


    def load_map_info(self):
        try:
            self.connect_to_database()

            query = "SELECT PARK_INFO.PK_NUM, PARK_INFO.PK_STATUS, CAR_INFO.CAR_NUM \
                    FROM PARK_INFO LEFT JOIN CAR_INFO ON PARK_INFO.PK_NUM = CAR_INFO.PK_NUM \
                    WHERE PARK_INFO.PK_NUM NOT LIKE 'R%'"
            cursor.execute(query)
            rows = cursor.fetchall()

            
            self.table_map_info.setRowCount(len(rows))
            for row_idx, row_data in enumerate(rows):
                for col_idx, col_data in enumerate(row_data):
                    if col_data is None:
                        item = QTableWidgetItem("Empty")
                    else:
                        item = QTableWidgetItem(str(col_data))
                    self.table_map_info.setItem(row_idx, col_idx, item)
            self.check_count += 1
            print(self.check_count)

        except mysql.connector.Error as e:
            print("Error")

    #======
    def back_to_login(self):
        if self.stk_manager.currentIndex() == 1:
            self.stk_manager.setCurrentIndex(0)
            self.le_ID.clear()
            self.le_PW.clear()

            while self.opened_window:
                window = self.opened_window.pop()
                window.close()
            
            self.opened_window = []
            self.page_login.setFocus()

    def exit_window(self):
        self.close()

    def remove_window_list(self, window):
        if window in self.opened_window:
            self.opened_window.remove(window)

    def check_value(self):
        print(self.opened_window)
            


#===added window===
class TotalGraph(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        loadUi("GUI/Admin/total_graph.ui", self)   #change path
        self.setWindowTitle("Graph")
        self.setFixedSize(750, 550)

        self.fig = plt.Figure()   #day_in
        self.canvas = FigureCanvas(self.fig)
        self.fig2 = plt.Figure()   #hour_in
        self.canvas2 = FigureCanvas(self.fig2)

        self.fig3 = plt.Figure()   #day_out
        self.canvas3 = FigureCanvas(self.fig3)
        self.fig4 = plt.Figure()   #hour_out
        self.canvas4 = FigureCanvas(self.fig4)

        self.layout_day_in.addWidget(self.canvas)
        self.layout_hour_in.addWidget(self.canvas2)

        self.layout_day_out.addWidget(self.canvas3)
        self.layout_hour_out.addWidget(self.canvas4)

        self.graph_draw()

    def graph_draw(self):   #test
        weekdata_x = ['Sun', 'Mon', 'Tus', 'Wed', 'Thu', 'Fri', 'Sat']
        daydata_x = ['morning', 'day', 'evening', 'night']
        weekdata_y_in = [5, 7, 6, 4, 3, 3, 2]
        weekdata_y_out = [4, 7, 7, 4, 3, 5, 3]
        daydata_y_in = [6, 3, 1, 1]
        daydata_y_out = [3, 2, 7, 4]
        #===day_in===
        ax = self.fig.add_subplot(111)
        ax.plot(weekdata_x, weekdata_y_in, label="")

        ax.set_ylim(0, 10)
        ax.legend()
        self.canvas.draw()

        #===hour_in===
        ax2 = self.fig2.add_subplot(111)
        ax2.bar(daydata_x, daydata_y_in, label="")
        
        ax2.set_ylim(0, 10)
        ax2.set_yticks(range(0, 10, 2))
        ax2.legend()
        self.fig2.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.2)
        self.canvas2.draw()


        #===day_out===
        ax3 = self.fig3.add_subplot(111)
        ax3.plot(weekdata_x, weekdata_y_out, label="")
        ax3.set_ylim(0, 10)

        ax3.legend()
        self.canvas3.draw()

        #===hour_out===
        ax4 = self.fig4.add_subplot(111)
        ax4.bar(daydata_x, daydata_y_out, label="")
        ax4.set_ylim(0, 10)
        ax4.set_yticks(range(0, 10, 2))
        ax4.legend()
        self.fig4.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.2)
        self.canvas4.draw()

    def closeEvent(self, event):
        self.parent().remove_window_list(self.parent().graph_window)
        self.parent().flag_open_graph = False
        super().closeEvent(event)


class MapInfo(QDialog, BDConnector):
    def __init__(self, parent=None):
        super().__init__(parent)
        loadUi("GUI/Admin/map_info.ui", self)   #change path
        self.setWindowTitle("Map")
        self.setFixedSize(720, 820)

        self.pixmap_map_image = QPixmap()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_timer_callback)
        self.update_timer.start(3000)

        self.map_update()

        self.count = 0
  

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            pos = event.pos()
            print(f"Clicked at: {pos}")
            
    def map_update(self):
        try:
            self.connect_to_database()

            query = "SELECT PK_NUM, CAR_NUM FROM CAR_INFO"
            cursor.execute(query)
            founds = cursor.fetchall()

            self.pixmap_map_image.load("GUI/Admin/park_map.png")   #change path
            self.pixmap_map_image = self.pixmap_map_image.scaled(self.lb_map.width(), self.lb_map.height())

            self.lb_map.setPixmap(self.pixmap_map_image)

            if founds:
                painter = QPainter(self.pixmap_map_image)
                
                if not painter.isActive():
                    painter.begin(self.pixmap_map_image)

                painter.setPen(QPen(Qt.blue, 5, Qt.SolidLine))
                font = QFont()
                font.setFamily("Times")
                font.setBold(True)
                font.setPointSize(15)
                painter.setFont(font)

                for pk_num, car_num in founds:
                    coords = park_geo.get(pk_num, "not found")
                    #print(f"PK_NUM:{pk_num}, CAR_NUM:{car_num}, Coordinates:{coords}")

                    if coords != "not found":
                        painter.drawText(coords[0], coords[1], car_num)
                    else:
                        print(f"Coordinates not found for PK_NUM:{pk_num}")

                self.charging_area(painter)
                    
                painter.end  
                self.lb_map.setPixmap(self.pixmap_map_image)

            else:
                print("Not Found")

        except mysql.connector.Error as e:
            print("Error")

    def charging_area(self, painter):   #충전 구역 표시
        c_points = [
            QPoint(132, 64),
            QPoint(30, 64),
            QPoint(30, 325),
            QPoint(132, 325)
        ]

        polygon = QPolygon(c_points)
        painter.setPen(QColor(0, 0, 0))
        painter.setBrush(QColor(0, 0, 255, 100))  # 반투명 파란색
        painter.drawPolygon(polygon)


    def update_timer_callback(self):
        self.map_update()
        
        print("update complete: ", self.count)
        self.count += 1
        

    def closeEvent(self, event):
        self.parent().remove_window_list(self.parent().map_window)
        self.parent().flag_open_map = False
        self.update_timer.stop()
        super().closeEvent(event)

class Server_To_Admin_Sub(Node):   #/server_and_admin sub
    def __init__(self):
        super().__init__("admin_gui")
        self.subscription = self.create_subscription(
            String,
            "/server_and_admin",
            self.robot_battery_listener_callback,
            10
        )
        self.subscription
        self.latest_msg = ""

    def robot_battery_listener_callback(self, msg):
        self.latest_msg = msg.data
        self.get_logger().info(f"Received: '{msg.data}'")

class Ros2Thread(QThread):
    rx_signal = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        rp.init()
        self.node = Server_To_Admin_Sub()
    
    def run(self):
        while rp.ok():
            rp.spin_once(self.node)
            if self.node.latest_msg:
                self.rx_signal.emit(self.node.latest_msg)



def main():
    app = QApplication(sys.argv)
    ros2_thread = Ros2Thread()
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



        

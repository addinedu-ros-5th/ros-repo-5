import sys
from PyQt5.QtCore import Qt, QPoint, QTimer
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
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("Hello, Qt!")
        self.setFixedSize(750, 550)

        self.table_check_flow.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_check_flow.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table_map_info.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_map_info.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self.pbt_exit.clicked.connect(self.exit_window)

        self.pbt_A.clicked.connect(self.open_total_graph)
        self.pbt_B.clicked.connect(self.open_map_info)
        self.pbt_D.clicked.connect(self.check_value)

        self.le_ID.returnPressed.connect(self.check_manager_info)
        self.le_PW.returnPressed.connect(self.check_manager_info)

        self.ctrl_e = QShortcut(QKeySequence("Ctrl+E"), self)
        self.ctrl_e.activated.connect(self.back_to_login)

        self.page_login.setFocus()
        self.opened_window = []

        self.flag_open_graph = False
        self.flag_open_map = False

        self.load_map_info()


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
                    FROM PARK_INFO LEFT JOIN CAR_INFO ON PARK_INFO.PK_NUM = CAR_INFO.PK_NUM"
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

        self.testdata_x = ['Sun', 'Mon', 'Tus', 'Wed', 'Thu', 'Fri', 'Sat']
        self.testdata_y = [2, 7, 6, 8, 3, 3, 4]

        self.graph_draw()

    def graph_draw(self):   #test
        #===day_in===
        x = self.testdata_x
        y = self.testdata_y

        ax = self.fig.add_subplot(111)
        ax.plot(x, y, label="sin")
        ax.set_xlabel("x")
        ax.set_xlabel("y")
        ax.set_title("my test graph")
        ax.legend()
        ax.set_ylim(0, 10)
        self.canvas.draw()

        #===hour_in===
        x2 = self.testdata_x
        y2 = self.testdata_y

        ax2 = self.fig2.add_subplot(111)
        ax2.bar(x2, y2, label="test2")
        # ax2.set_xlabel("hour")
        # ax2.set_ylabel("car")
        # ax2.set_title("hour in")
        ax2.legend()
        ax2.set_ylim(0, 10)
        self.fig2.tight_layout()
        self.fig2.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.2)
        self.canvas2.draw()


        #===day_out===
        x3 = np.arange(0, 50, 1)
        y3 = np.sin(x3)

        ax3 = self.fig3.add_subplot(111)
        ax3.plot(x3, y3, label="sin")
        ax3.set_xlabel("x3")
        ax3.set_xlabel("y3")

        ax3.set_title("my sin graph")
        ax3.legend()
        self.canvas3.draw()

        #===hour_out===
        x4 = self.testdata_x
        y4 = self.testdata_y

        ax4 = self.fig4.add_subplot(111)
        ax4.bar(x4, y4, label="test24")
        ax4.set_xlabel("hour")
        ax4.set_ylabel("car")
        ax4.set_title("hour out")
        ax4.legend()
        ax4.set_ylim(0, 10)
        self.canvas4.draw()

    def closeEvent(self, event):
        #self.parent().show()
        #self.parent().move(self.pos())
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
        #self.charging_area()

        

        

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
                print("OK")
                print(founds)
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
                    print(f"PK_NUM:{pk_num}, CAR_NUM:{car_num}, Coordinates:{coords}")

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
        #self.parent().show()
        #self.parent().move(self.pos())
        self.parent().remove_window_list(self.parent().map_window)
        self.parent().flag_open_map = False
        self.update_timer.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())



        

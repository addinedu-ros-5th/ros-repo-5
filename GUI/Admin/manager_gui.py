import sys
from PyQt5.QtCore import Qt, QPoint
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

#다음 목표:그래프 그릴 수 있도록 틀 잡아두기(절반만 수행)
#       :좌표 구하기?
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




from_class = uic.loadUiType("GUI/Admin/manager_gui.ui")[0]
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

        self.load_map_info()


    #===open window function===
    def open_total_graph(self):
        #self.hide()
        self.graph_window = TotalGraph(self)
        self.graph_window.move(self.pos())
        self.graph_window.show()
        self.opened_window.append(self.graph_window)

    def open_map_info(self):
        #self.hide()
        self.map_window = MapInfo(self)
        self.map_window.move(self.pos())
        self.map_window.show()
        self.opened_window.append(self.map_window)


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
        loadUi("GUI/Admin/total_graph.ui", self)
        self.setWindowTitle("Graph")
        self.setFixedSize(750, 550)

        self.fig = plt.Figure()
        self.canvas = FigureCanvas(self.fig)
        self.fig3 = plt.Figure()
        self.canvas3 = FigureCanvas(self.fig3)

        self.layout_test.addWidget(self.canvas)
        self.layout_test3.addWidget(self.canvas3)

        self.graph_draw()

    def graph_draw(self):   #test
        x = np.arange(0, 100, 1)
        y = np.sin(x)

        ax = self.fig.add_subplot(111)
        ax.plot(x, y, label="sin")
        ax.set_xlabel("x")
        ax.set_xlabel("y")

        ax.set_title("my sin graph")
        ax.legend()
        self.canvas.draw()

        x3 = np.arange(0, 50, 1)
        y3 = np.sin(x3)

        ax3 = self.fig3.add_subplot(111)
        ax3.plot(x3, y3, label="sin")
        ax3.set_xlabel("x3")
        ax3.set_xlabel("y3")

        ax3.set_title("my sin graph")
        ax3.legend()
        self.canvas3.draw()

    def closeEvent(self, event):
        #self.parent().show()
        #self.parent().move(self.pos())
        self.parent().remove_window_list(self.parent().graph_window)
        super().closeEvent(event)


class MapInfo(QDialog, BDConnector):
    def __init__(self, parent=None):
        super().__init__(parent)
        loadUi("GUI/Admin/map_info.ui", self)
        self.setWindowTitle("Map")
        self.setFixedSize(720, 820)

        self.pixmap_map_image = QPixmap()
        self.pixmap_map_image.load("GUI/Admin/park_map.png")
        self.pixmap_map_image = self.pixmap_map_image.scaled(self.lb_map.width(), self.lb_map.height())
        
        self.lb_map.setPixmap(self.pixmap_map_image)
        self.map_update()
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
            QPoint(120, 50),
            QPoint(18, 50),
            QPoint(18, 320),
            QPoint(120, 320)
        ]

        polygon = QPolygon(c_points)
        painter.setPen(QColor(0, 0, 0))
        painter.setBrush(QColor(0, 0, 255, 127))  # 반투명 파란색
        painter.drawPolygon(polygon)
        

        

    def closeEvent(self, event):
        #self.parent().show()
        #self.parent().move(self.pos())
        self.parent().remove_window_list(self.parent().map_window)
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())


# self.fig_week_in = plt.Figure()
#         self.canvas_week_in = FigureCanvas(self.fig_week_in)
#         self.fig_hour_in = plt.Figure()
#         self.canvas_hour_in = FigureCanvas(self.fig_hour_in)

#         self.fig_week_out = plt.Figure()
#         self.canvas_week_out = FigureCanvas(self.fig_week_out)
#         self.fig_hour_out = plt.Figure()
#         self.canvas_hour_out = FigureCanvas(self.fig_hour_out)


#         self.fig = plt.Figure()
#         self.canvas = FigureCanvas(self.fig)
#         self.fig3 = plt.Figure()
#         self.canvas3 = FigureCanvas(self.fig3)

#         self.layout_week_in.addWidget(self.canvas_week_in)
#         self.layout_hour_in.addWidget(self.canvas_hour_in)

#         self.layout_week_out.addWidget(self.canvas_week_out)
#         self.layout_hour_out.addWidget(self.canvas_hour_out)

# layout name: layout_week_in, layout_week_out, layout_hour_in, layout_hour_out

        

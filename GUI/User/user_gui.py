import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.uic import loadUi
#from PyQt5.QtWidgets import QWidget

from_class = uic.loadUiType("GUI/User/user_gui.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("Hello, Qt!")
        self.setFixedSize(750, 550)

        self.car_num_text = ""
        self.selected_num = ""

        self.timer1 = QTimer(self)
        self.timer2 = QTimer(self)
        

        button_list = [self.pbt_num1, self.pbt_num2, self.pbt_num3, self.pbt_num4, self.pbt_num5,
                       self.pbt_num6, self.pbt_num7, self.pbt_num8, self.pbt_num9, self.pbt_undo]
        for button in button_list:
            button.setFixedSize(95, 95)
        self.pbt_num0.setFixedSize(197, 95)

        self.car_num_data = [   #test
            "11가 1123",
            "22나 3456",
            "33다 3456",
            "445라 9898"
        ]

        #===stack0:page_input_num===
        self.pbt_num0.clicked.connect(lambda: self.input_num_from_pbt(0))
        self.pbt_num1.clicked.connect(lambda: self.input_num_from_pbt(1))
        self.pbt_num2.clicked.connect(lambda: self.input_num_from_pbt(2))
        self.pbt_num3.clicked.connect(lambda: self.input_num_from_pbt(3))
        self.pbt_num4.clicked.connect(lambda: self.input_num_from_pbt(4))
        self.pbt_num5.clicked.connect(lambda: self.input_num_from_pbt(5))
        self.pbt_num6.clicked.connect(lambda: self.input_num_from_pbt(6))
        self.pbt_num7.clicked.connect(lambda: self.input_num_from_pbt(7))
        self.pbt_num8.clicked.connect(lambda: self.input_num_from_pbt(8))
        self.pbt_num9.clicked.connect(lambda: self.input_num_from_pbt(9))
        self.pbt_undo.clicked.connect(self.remove_num)
        
        self.timer1.timeout.connect(self.tmove_page_check)
        self.timer2.timeout.connect(self.tmove_page_select)

        #===stack1:page_check===
        self.pbt_check_cancle.clicked.connect(lambda: self.back_page_input_num(0))
        self.pbt_out_signal.clicked.connect(lambda: self.back_page_input_num(1))

        
        #===stack2:page_select===
        self.table_select_car.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_select_car.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table_select_car.cellClicked.connect(self.select_to_check)
        self.pbt_select_cancle.clicked.connect(lambda: self.back_page_input_num(2))
        

        





    
    def remove_num(self):
        self.car_num_text = self.car_num_text[:-1]
        

        if self.car_num_text == "":
            print("not data")
            self.lb_input_carnum.setText("input car num")   #thinking
        else:
            print(self.car_num_text)
            self.lb_input_carnum.setText(self.car_num_text)
        
        
        

    def input_num_from_pbt(self, num):
        if len(self.car_num_text) < 4:
            self.car_num_text += str(num)
        print(self.car_num_text)
        self.lb_input_carnum.setText(self.car_num_text)

        if len(self.car_num_text) == 4:
            find_num_box = []
            find_count = 0
            for nums in self.car_num_data:
                if self.car_num_text in nums:
                    #print(nums)
                    find_num_box.append(nums)
                    find_count += 1

            if not find_num_box:
                print("not")
            if find_count == 1:
                self.selected_num = find_num_box[0]
                print(self.selected_num)
                self.timer1.start(500)
            if find_count >= 2:
                self.timer2.start(500)
                self.table_select_car.setRowCount(len(find_num_box))
                self.table_select_car.setColumnCount(1)

                for row_idx, data in enumerate(find_num_box):
                    self.table_select_car.setItem(row_idx, 0, QTableWidgetItem(data))
                    


    def select_to_check(self, row, column):
        self.selected_num = self.table_select_car.item(row, column).text()
        print(f"ROW {row} clicked: {self.selected_num}")
        self.stk_user.setCurrentIndex(1)
        self.lb_carnum.setText(self.selected_num)


                    
    def tmove_page_check(self):
        self.timer1.stop()
        self.stk_user.setCurrentIndex(1)
        self.lb_input_carnum.clear()
        self.car_num_text = ""
        self.lb_carnum.setText(self.selected_num)

    def tmove_page_select(self):
        self.timer2.stop()
        self.stk_user.setCurrentIndex(2)
        self.lb_input_carnum.clear()
        self.car_num_text = ""
        self.lb_carnum.setText(self.selected_num)
        

    def back_page_input_num(self, status):   #test
        if status == 0:
            self.stk_user.setCurrentIndex(0)
            self.selected_num = ""
            self.lb_carnum.clear()
            print("Reset")

        elif status == 1:
            self.stk_user.setCurrentIndex(0)
            self.selected_num = ""
            self.lb_carnum.clear()
            print("Signal")

        elif status == 2:
            self.stk_user.setCurrentIndex(2)
            print("back select")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())
import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

import mysql.connector
from mysql.connector import Error
from server_and_gui_package.DB_set import DB_SET

import re

class DBconnector:
    def __init__(self):
        self.connection = None
        self.cursor = None
    
    def connect_to_database(self):   #connect DB
        try:
            self.connection = mysql.connector.connect(
                host=DB_SET['HOST'],
                database=DB_SET['DATABASE'],
                user=DB_SET['USER'],
                password=DB_SET['PASSWORD']
            )

            if self.connection.is_connected():
                self.cursor = self.connection.cursor()
                print("MySQL Database connection successful")
                return True

        except mysql.connector.Error as e:
            print(f"Error: {e}")
            self.connection, self.cursor = None, None
            return False


class Server_And_User(Node, DBconnector):
    def __init__(self):
        Node.__init__(self, 'server_and_user_node')
        DBconnector.__init__(self)

        if self.connect_to_database():   #DB connect check
            self.get_logger().info("Connected to database")
        else:
            self.get_logger().error("Failed to connect to database")

        #===sub===
        self.user_to_server_sub = self.create_subscription(   #receive data from user gui
            String,
            '/user_to_server',
            self.user_gui_listener_callback,
            10)
        self.user_to_server_sub




        '''
        self.park_num_sub = self.create_subscription(   #receive data from another server
            String,
            "/send_park_num",
            self.receive_park_num,
            10)
        self.park_num_sub
        '''


        self.rfid_signal_sub = self.create_subscription(   #receive rfid signal
            String,
            "/out_time_topic",
            self.out_time_listner_callback,
            10)
        self.rfid_signal_sub


        #---add---
        self.park_num_10_sub = self.create_subscription(
            String,
            "/send_park_num_10",   #change and test
            self.park_num_10_listener_callback,
            10)
        self.park_num_10_sub

        self.park_num_23_sub = self.create_subscription(
            String,
            "/send_park_num_23",
            self.park_num_23_listener_callback,
            10)
        self.park_num_23_sub

        self.park_num_68_sub = self.create_subscription(
            String,
            "/send_park_num_68",
            self.park_num_68_listener_callback,
            10)
        self.park_num_68_sub

        #---add---
        #===sub===
        #===pub===
        self.server_to_user_pub = self.create_publisher(String, "/server_to_user", 10)   #send data to user gui
        self.park_num_pub = self.create_publisher(String, "/send_park_num", 10)   #send data to another server

        #===pub===
        #===variable===
        self.flag_search_num = False   #send data:car_num to user gui flag



        self.cpk_num_data = ""   #cpk_num_list.append(cpk_num_data)
        self.cpk_num_list = []   #to use car num and park num(no DB)

        self.wait_line_G = []   #waiting GX
        self.wait_line_F = []   #waiting FX
        self.wait_line_O = []   #waiting OX

        self.park_num_data = ""   #to send park_num of req_ready car's
        self.timeout_timer_pm = None


        self.history_ready_list = []
        #===variable===

    #===callback===
    def user_gui_listener_callback(self, msg):   #/user_to_server
        self.get_logger().info(f"received data from user gui: {msg.data}")   #print_log
        received_data = msg.data
        send_user_msg = String()   #server to user msg:data/signal
        send_park_num_msg = String()   #server to server msg:park num


        if self.connection and self.cursor:   #DB connect check
            try:
                if received_data[0] == "D":   #Data
                    car_4num = received_data[1:]   #("D" + car_4num)
                    query = "SELECT CAR_NUM, PK_NUM FROM CAR_INFO WHERE CAR_NUM LIKE %s"   #search car_4num -> DB
                    self.cursor.execute(query, (f"%{car_4num}%",))
                    find_results = self.cursor.fetchall()   #data check ok ex.[('123노3333', 'D2'), ('180호3333', 'D3')]


                    if len(find_results) == 1:   #only one
                        for row in find_results:
                            self.get_logger().info(f"Row: {row}")   #print_log
                            car_full_num = row[0]   #CAR_INFO.CAR_NUM

                            if self.flag_search_num == False:   #to send car num once
                                send_user_msg.data = "D" + car_full_num   #ex. D77가7777
                                self.server_to_user_pub.publish(send_user_msg)
                                self.flag_search_num = True
                            #else:                            
                            self.park_num_data = row[1]   #to send park num(/send_park_num)
                            self.cpk_num_data = str(row[0]) + ", " + str(row[1])   #save "car num, pk num"


                    elif len(find_results) >= 2:   #more than 2
                        for row in find_results:
                            self.get_logger().info(f"Row: {row}")   #print_log
                            car_full_num = row[0]   #CAR_INFO.CAR_NUM
                            send_user_msg.data = "D" + car_full_num
                            self.server_to_user_pub.publish(send_user_msg)
                            self.flag_search_num = True
                    else:
                        self.get_logger().info("can't find car num")


                elif received_data[0] == "S":   #Signal
                    self.flag_search_num = False
                    signal_from_user = received_data[1:]   #("S" + signal)


                    if signal_from_user == "req_ready":   #push pbt_out_signal(ready) -> move to GX
                        self.cpk_num_list.append(self.cpk_num_data)
                        query = "SELECT PK_NUM, PK_STATUS FROM PARK_INFO WHERE PK_NUM LIKE 'G%'"   #find park num G and status
                        self.cursor.execute(query)
                        find_results = self.cursor.fetchall()   #ex. [('G1', 'F'), ('G2', 'F')]
                        goal_park_num = None   #RXXTOO

                        if find_results[0][1] == "F":   #G1 = "F"
                            goal_park_num = find_results[0][0]   #G1
                        elif find_results[0][1] != "F" and find_results[1][1] == "F":   #G1 = "T", G2 = "F"
                            goal_park_num = find_results[1][0]   #G2
                        else:   #G1 = "T", G2 = "T"
                            goal_park_num = "GX"
                            print("Area G is Full!")

                        query = "SELECT PK_NUM, PK_STATUS FROM PARK_INFO WHERE PK_NUM='E2'"
                        self.cursor.execute(query)
                        find_result = self.cursor.fetchone()   #ex. ('E2', 'T')
                        print(find_result)
                        if self.park_num_data == "A2" and find_result[1] == 'T':
                            send_park_num_msg.data = "R" + self.park_num_data + "T" + goal_park_num + "M"   #RA2TG1M
                            print("OOOOO: ", send_park_num_msg.data)
                            query = "SELECT CAR_NUM, PK_NUM FROM CAR_INFO WHERE PK_NUM='E2'"
                            self.cursor.execute(query)
                            find_result = self.cursor.fetchone()
                            step1 = find_result[0] + ", " + find_result[1]
                            self.cpk_num_list.append(step1)
                            print(step1)
                        else:
                            send_park_num_msg.data = "R" + self.park_num_data + "T" + goal_park_num
    
                        self.park_num_pub.publish(send_park_num_msg)

                        if goal_park_num == "GX":
                            self.wait_line_G.append(send_user_msg.data)
                        elif goal_park_num != "GX" and send_park_num_msg.data[-1] != "M":
                            query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"   #DB:PARK_INFO.PK_STATUS update
                            self.cursor.execute(query, ('F', self.park_num_data,))   #R status:T -> F
                            self.cursor.execute(query, ('T', goal_park_num,))   #T status:F -> T
                            self.connection.commit()
                        elif goal_park_num != "GX" and send_park_num_msg.data[-1] == "M":
                            query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"   #DB:PARK_INFO.PK_STATUS update
                            self.cursor.execute(query, ('F', 'E2',))   #R status:T -> F   ex. A2, E2
                            self.cursor.execute(query, ('T', goal_park_num,))   #T status:F -> T   ex. G1
                            self.connection.commit()

                        else:
                            pass


                    elif signal_from_user[:7] == "req_out":   #ex. Sreq_outN77호7777
                        if self.timeout_timer:   #timer cancel
                            self.timeout_timer.cancel()
                        car_num = signal_from_user[8:]   #only full car num
                        for each in self.cpk_num_list:
                            if car_num in each:
                                step1 = each
                        step2 = re.split(", ", step1)
                        update_park_num = step2[1]   #pk num
                        query = "SELECT PK_NUM, PK_STATUS FROM PARK_INFO WHERE PK_NUM='O1'"
                        self.cursor.execute(query)
                        find_results = self.cursor.fetchall()   #ex. ('O1', 'F')
                        goal_park_num = None

                        if find_results[0][1] == "F":   #O1 = "F"
                            goal_park_num = find_results[0][0]
                        else:
                            goal_park_num = "OX"
                            print("Area O is Full!")

                        send_park_num_msg.data = "R" + update_park_num + "T" + goal_park_num   #RG1TO1 or RG1TF1(timeout)
                        self.park_num_pub.publish(send_park_num_msg)
                        if goal_park_num == "OX":
                            self.wait_line_O.append(send_park_num_msg.data)
                        else:
                            query = "UPDATE CAR_INFO SET PK_NUM=%s WHERE CAR_NUM=%s"
                            self.cursor.execute(query, (goal_park_num, car_num,))

                            for each in self.cpk_num_list:
                                if car_num in each:
                                    #step1 = each
                                    self.cpk_num_list = [item for item in self.cpk_num_list if item != each]
                            query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                            self.cursor.execute(query, ('F', update_park_num,))
                            self.cursor.execute(query, ('T', goal_park_num,))
                            self.connection.commit()

                            if self.wait_line_G and self.wait_line_G[0]:
                                step1 = self.wait_line_G[0]
                                self.wait_line_G.pop(0)
                                wait_start_park_num = step1[1:3]
                                send_park_num_msg.data = "R" + wait_start_park_num + "T" + update_park_num   #pr:RA2TGX -> RA2TG1
                                self.park_num_pub.publish(send_park_num_msg)

                                query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                                self.cursor.execute(query, ('F', wait_start_park_num,))
                                self.cursor.execute(query, ('T', update_park_num,))
                                self.connection.commit()


                    elif signal_from_user == "test":
                        send_park_num_msg.data = "GOGO"
                        self.park_num_pub.publish(send_park_num_msg)


                    else:   #req_cancle
                        print("req_cancle")


            except Error as e:
                self.get_logger().error("DB error in user_gui_listener_callback")
        


    #---add---
    def park_num_10_listener_callback(self, msg):
        print("A")
        received_data = msg.data
        rb_id = "R1"
        self.receive_park_num(rb_id, received_data)



    def park_num_23_listener_callback(self, msg):
        print("B")
        received_data = msg.data
        rb_id = "R2"
        self.receive_park_num(rb_id, received_data)

    def park_num_68_listener_callback(self, msg):
        print("C")
        received_data = msg.data
        rb_id = "R3"
        self.receive_park_num(rb_id, received_data)
    #---add---
    #===callback===
    #===function===
    def receive_park_num(self, rb_id, data):   #/send_park_num
        received_data = data
        rb_id = rb_id   #현재 사용처 없음
        send_user_msg = String()

        if self.connection and self.cursor:
            try:
                if received_data[0] == "S":   #Start(~end)
                    start_park_num = received_data[1:3]   #S
                    end_park_num = received_data[4:6]   #T
                    for each in self.cpk_num_list:
                        if start_park_num in each:
                            step1 = each
                            self.cpk_num_list = [item for item in self.cpk_num_list if item != each]
                    step2 = re.split(", ", step1)
                    update_car_num = step2[0]

                    send_user_msg.data = "D" + update_car_num + "R"
                    if send_user_msg.data not in self.history_ready_list:   #except same num
                        self.history_ready_list.append(send_user_msg.data)
                        if end_park_num == "G1" or end_park_num == "G2":
                            self.server_to_user_pub.publish(send_user_msg)
                        else:
                            print("T: not G1, G2")

                    query = "UPDATE CAR_INFO SET PK_NUM=%s WHERE CAR_NUM=%s"   #DB:CAR_INFO update
                    self.cursor.execute(query, (end_park_num, update_car_num,))
                    self.connection.commit()

                    step1 = update_car_num + ", " + end_park_num
                    self.cpk_num_list.append(step1)
                    if end_park_num == "G1" or end_park_num == "G2":
                        self.set_timer(end_park_num)
                        #end_park_num = None



            except Error as e:
                self.get_logger().error("DB error in receive_park_num")





    #===function===
            
    def set_timer(self, pk_num):   #setting timeout_timer
        self.timeout_timer_pm = pk_num
        self.timeout_timer = self.create_timer(10.0, self.timeout_timer_callback)

    def timeout_timer_callback(self):   #timeout_timer callback
        send_park_num_msg = String()
        timeout_pk_num = self.timeout_timer_pm
        if self.timeout_timer:
            self.timeout_timer.cancel()
        if self.connection and self.cursor:
            try:
                query = "SELECT PK_NUM, PK_STATUS FROM PARK_INFO WHERE PK_NUM LIKE 'F%'"   #find park num F and status
                self.cursor.execute(query)
                find_results = self.cursor.fetchall()
                goal_park_num = None

                if find_results[0][1] == "F":   #F1 = "F"
                    goal_park_num = find_results[0][0]   #F1
                elif find_results[0][1] != "F" and find_results[1][1] == "F":   #F1 = "T", F2 = "F"
                    goal_park_num = find_results[1][0]   #F2
                else:
                    goal_park_num = "FX"
                    print("Area F is Full!")
                send_park_num_msg.data = "R" + timeout_pk_num + "T" + goal_park_num
                self.park_num_pub.publish(send_park_num_msg)   #RG1TF1

                if goal_park_num == "FX":
                    self.wait_line_F.append(send_park_num_msg.data)
                else:
                    for each in self.cpk_num_list:
                        if timeout_pk_num in each:
                            step1 = each
                            self.cpk_num_list = [item for item in self.cpk_num_list if item != each]
                    step2 = re.split(", ", step1)
                    update_car_num = step2[0]

                    query = "UPDATE CAR_INFO SET PK_NUM=%s WHERE CAR_NUM=%s"
                    self.cursor.execute(query, (goal_park_num, update_car_num,))

                    query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                    self.cursor.execute(query, ('F', timeout_pk_num,))
                    self.cursor.execute(query, ('T', goal_park_num,))
                    self.connection.commit()

                    step3 = update_car_num + ", " + goal_park_num
                    self.cpk_num_list.append(step3)

                    if self.wait_line_G and self.wait_line_G[0]:
                        step1 = self.wait_line_G[0]
                        self.wait_line_G.pop(0)
                        wait_start_park_num = step1[1:3]
                        send_park_num_msg.data = "R" + wait_start_park_num + "T" + timeout_pk_num   #pr:RA2TGX -> RA2TG1
                        self.park_num_pub.publish(send_park_num_msg)

                        query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                        self.cursor.execute(query, ('F', wait_start_park_num,))
                        self.cursor.execute(query, ('T', timeout_pk_num,))
                        self.connection.commit()

            except Error as e:
                self.get_logger().error("DB error in timeout_timer_callback")


    #==================================
    def out_time_listner_callback(self, msg):
        received_data = msg.data
        send_park_num_msg = String()

        if self.connection and self.cursor:
            try:
                if received_data:  #ok
                    query = "SELECT CAR_NUM FROM CAR_INFO WHERE PK_NUM='O1'"
                    self.cursor.execute(query)
                    find_result = self.cursor.fetchone()
                    out_car_num = find_result[0]

                    query = "DELETE FROM CAR_INFO WHERE CAR_NUM=%s"
                    self.cursor.execute(query, (out_car_num,))
                    query = "UPDATE PARK_INFO SET PK_STATUS='F' WHERE PK_NUM='O1'"
                    self.cursor.execute(query)
                    self.connection.commit()

                    if self.wait_line_O and self.wait_line_O[0]:
                        step1 = self.wait_line_O[0]
                        self.wait_line_O.pop(0)
                        wait_start_park_num = step1[1:3]
                        send_park_num_msg.data = "R" + wait_start_park_num + "T" + "O1"   #RG2TOX -> RG2TO1
                        self.park_num_pub.publish(send_park_num_msg)

                        query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                        self.cursor.execute(query, ('F', wait_start_park_num,))
                        self.cursor.execute(query, ('F', 'O1'))
                        self.connection.commit()

                    for each in self.cpk_num_list:
                        if out_car_num in each:
                            step1 = each
                            self.cpk_num_list = [item for item in self.cpk_num_list if item != each]

            except Error as e:
                self.get_logger().error("DB error in out_time_listner_callback")

    #==================================


    def __del__(self):
        if self.connection and self.connection.is_connected():
            self.cursor.close()
            self.connection.close()
            print("MySQL connection is closed")

def main(args=None):
    rp.init(args=args)
    server_and_user = Server_And_User()
    
    rp.spin(server_and_user)
    server_and_user.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
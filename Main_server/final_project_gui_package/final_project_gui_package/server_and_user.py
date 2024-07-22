import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

import mysql.connector
from mysql.connector import Error
from final_project_gui_package.DB_set import DB_SET

import re

class BDConnector:
    def __init__(self):
        self.connection = None
        self.cursor = None
    
    def connect_to_database(self):
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

class Server_And_User(Node, BDConnector):
    def __init__(self):
        Node.__init__(self, 'test9898')
        BDConnector.__init__(self)

        if self.connect_to_database():
            self.get_logger().info("Connected to database")
        else:
            self.get_logger().error("Failed to connect to database")

        self.user_to_server_sub = self.create_subscription(
            String,
            '/user_to_server',
            self.user_gui_listener_callback,
            10)
        self.user_gui_listener_callback

        self.park_num_sub = self.create_subscription(
            String,
            "/send_park_num",
            self.park_num_listener_callback,
            10)
        self.park_num_sub

        self.server_to_user_pub = self.create_publisher(String, "/server_to_user", 10)
        self.park_num_pub = self.create_publisher(String, "/send_park_num", 10)


        self.flag_search_num = False

        self.cpk_num_text = ""
        self.cpk_num_list = []   #to use car num and park num(no DB)
        self.park_text = ""

    def user_gui_listener_callback(self, msg):
        self.get_logger().info(f"received user gui data: {msg.data}")
        self.received_user_data = msg.data
        send_user_text_msg = String()   #server to user msg
        send_park_text_msg = String()
        

        if self.received_user_data[0] == "D" and self.connection and self.cursor:   #data
            try:
                car_4num = self.received_user_data[1:]
                query = "SELECT * FROM CAR_INFO WHERE CAR_NUM LIKE %s"
                self.cursor.execute(query, (f"%{car_4num}%",))
                find_results = self.cursor.fetchall()

                if len(find_results) == 1:
                    for row in find_results:
                        self.get_logger().info(f"Row: {row}")
                        send_user_text = str(row[0])
                        if self.flag_search_num == False:
                            send_user_text_msg.data = "D" + send_user_text
                            self.server_to_user_pub.publish(send_user_text_msg)
                            self.flag_search_num = True
                        
                        self.park_text = str(row[3])   #for /send_park_num
                        self.cpk_num_text = str(row[0]) + ", " + str(row[3])

                elif len(find_results) >= 2:
                    for row in find_results:
                        self.get_logger().info(f"Row: {row}")
                        send_user_text = row[0]
                        send_user_text_msg.data = "D" + send_user_text
                        self.server_to_user_pub.publish(send_user_text_msg)
                        self.flag_search_num = True

                else:
                    self.get_logger().info("not found")

            except Error as e:
                self.get_logger().error("Database error")
            

        elif self.received_user_data[0] == "S":   #Signal
            self.flag_search_num = False
            user_signal = self.received_user_data[1:]

            if user_signal == "req_ready":
                print("receive: req_ready")
                self.cpk_num_list.append(self.cpk_num_text)
                print(self.cpk_num_list)
                send_park_text_msg.data = "R" + self.park_text
                self.park_num_pub.publish(send_park_text_msg)

                if self.connection and self.cursor:   #park_info update:'T' -> 'F'
                    try:
                        query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                        self.cursor.execute(query, ('F', self.park_text,))   #ok
                        self.connection.commit()


                    except Error as e:
                        self.get_logger().error("Database update:1 error")



            elif user_signal == "req_out":
                print("receive: req_out")

            elif user_signal == "req_cancle":
                print("receive: req_cancle")

            

    def park_num_listener_callback(self, msg):
        self.recevied_park_data = msg.data
        send_user_text_msg = String()
        if self.recevied_park_data[0] == "S":
            start_park_num = self.recevied_park_data[1:3]
            end_park_num = self.recevied_park_data[4:6]
            #print(start_park_num, end_park_num)   #ok
            for each in self.cpk_num_list:
                if start_park_num in each:
                    step1 = each
                    self.cpk_num_list = [item for item in self.cpk_num_list if item != each]
            print(step1)   #ok
            step2 = re.split(", ", step1)
            update_car_num = step2[0]
            send_user_text_msg.data = "D" + update_car_num + "R"
            self.server_to_user_pub.publish(send_user_text_msg)
            print("send_user_text_msg.data: ", send_user_text_msg.data)


            if self.connection and self.cursor:
                try:
                    query = "UPDATE CAR_INFO SET PK_NUM=%s WHERE CAR_NUM=%s"
                    self.cursor.execute(query, (end_park_num, update_car_num,))
                    query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                    self.cursor.execute(query, ('T', end_park_num,))
                    self.connection.commit()

                    step1 = update_car_num + ", " + end_park_num   #add new status(DB:select X)
                    self.cpk_num_list.append(step1)
                    print("add list", self.cpk_num_list)   #ok

                except Error as e:
                    self.get_logger().error("Database update:2 error")


        elif self.recevied_park_data[0] == "T":   #아직 테스트 안됨
            res_park_num = self.recevied_park_data[1:]



            


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
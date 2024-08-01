import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

import mysql.connector
from mysql.connector import Error
from server_and_gui_package.DB_set import DB_SET

import re

class BDConnector:
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


class In_Park_Area(Node, BDConnector):
    def __init__(self):
        Node.__init__(self, 'come_in_park_area_node')
        BDConnector.__init__(self)

        if self.connect_to_database():
            self.get_logger().info("Connected to database")
        else:
            self.get_logger().error("Failed to connect to database")


        self.rfid_signal_sub = self.create_subscription(   #receive rfid signal
            String,
            "/in_time_topic",
            self.in_time_listner_callback,
            10)
        self.rfid_signal_sub

        self.empty_park_num_sub = self.create_subscription(   #receive empty park num
            String,
            "/find_empty_park_num",
            self.empty_park_num_listner_callback,
            10)
        self.empty_park_num_sub

        self.empty_park_num_pub = self.create_publisher(String, "/find_empty_park_num", 10)   #send empty park num


        self.in_car_list = []   #in car num list("car_num, goal_park_num", )




    def in_time_listner_callback(self, msg):
        received_data = msg.data
        send_park_text_msg = String()

        if self.connection and self.cursor:
            try:
                if received_data:
                    query = "SELECT CAR_NUM FROM CAR_INFO WHERE PK_NUM='I1'"
                    self.cursor.execute(query)
                    step1 = self.cursor.fetchone()
                    in_car_num = step1[0]

                    select_query_pm = ['A2', 'B2', "E2"]
                    query = "SELECT PK_NUM, PK_STATUS FROM PARK_INFO WHERE PK_NUM LIKE %s OR PK_NUM LIKE %s OR PK_NUM LIKE %s"
                    self.cursor.execute(query, (*select_query_pm,))
                    find_results = self.cursor.fetchall()
                    print(find_results)   #OK

                    goal_park_num = ""
                    if find_results[0][1] == 'F':   #A2 = 'F'
                        goal_park_num = find_results[0][0]

                    elif find_results[1][1] == 'F':   #A2 = 'T, B2 = 'F'
                        goal_park_num = find_results[1][0]

                    else:   #A2 = 'T, B2 = 'T'
                        goal_park_num = find_results[2][0]


                    if goal_park_num:
                        in_car_num_text = in_car_num + ", " + goal_park_num
                        self.in_car_list.append(in_car_num_text)

                        send_park_text_msg.data = "R" + "I1" + "T" + goal_park_num
                        self.empty_park_num_pub.publish(send_park_text_msg)

                        query = "UPDATE PARK_INFO SET PK_STATUS=%s WHERE PK_NUM=%s"
                        self.cursor.execute(query, ('F', 'I1',))   #I1 = 'F'
                        self.cursor.execute(query, ('T', goal_park_num,))
                        self.connection.commit()

                    else:
                        print("The Park Area is Full!")



            except Error as e:
                self.get_logger().error("DB error in rfid_signal_listner_callback")

    def empty_park_num_listner_callback(self, msg):
        received_data = msg.data

        if self.connection and self.cursor:
            try:
                if received_data[0] == "S":
                    start_park_num = received_data[1:3]
                    end_park_num = received_data[4:6]

                    for each in self.in_car_list:
                        if end_park_num in each:
                            step1 = each
                            self.in_car_list = [item for item in self.in_car_list if item != each]


                    step2 = re.split(", ", step1)   #take car num to update
                    update_car_num = step2[0]

                    query = "UPDATE CAR_INFO SET PK_NUM=%s WHERE CAR_NUM=%s"
                    self.cursor.execute(query, (end_park_num, update_car_num,))
                    self.connection.commit()


            except Error as e:
                self.get_logger().error("DB error in empty_park_num_listner_callback")

        




def main(args=None):
    rp.init(args=args)
    in_park_area = In_Park_Area()

    rp.spin(in_park_area)
    in_park_area.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()

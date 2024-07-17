import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

import mysql.connector
from mysql.connector import Error
from final_project_gui_package.DB_set import DB_SET

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

class Take_DB_Data(Node, BDConnector):
    def __init__(self):
        Node.__init__(self, 'test1212')
        BDConnector.__init__(self)

        if self.connect_to_database():
            self.get_logger().info("Connected to database")
        else:
            self.get_logger().error("Failed to connect to database")

        self.subscription = self.create_subscription(
            String,
            '/from_gui_signal',  # 여기에 발행된 토픽 이름을 입력해야 합니다.
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.car_num_publisher = self.create_publisher(String, "/from_ros2_signal", 10)
        self.park_num_publisher = self.create_publisher(String, "/send_park_num", 10)

        self.flag_num = False
        self.park_num_msg = String()

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        self.from_gui_data = msg.data
        if self.from_gui_data != "out_signal" and self.from_gui_data != "cancled" and self.connection and self.cursor:
            try:
                query = "SELECT * FROM CAR_INFO WHERE CAR_NUM LIKE %s"
                self.cursor.execute(query, (f"%{self.from_gui_data}%",))
                find_results = self.cursor.fetchall()
                if len(find_results) == 1:
                    for row in find_results:
                        car_num_msg = String()
                        self.get_logger().info(f"Row: {row}")
                        car_num_msg.data = str(row[0])
                        if self.flag_num == False:
                            self.car_num_publisher.publish(car_num_msg)
                            self.flag_num = True
                        self.park_num_msg.data = str(row[3])

                elif len(find_results) >= 2:
                    for row in find_results:
                        car_num_msg = String()
                        self.get_logger().info(f"Row: {row}")
                        car_num_msg.data = str(row[0])
                        self.car_num_publisher.publish(car_num_msg)
                        self.flag_num = True
                        
                else:
                    self.get_logger().info("Not found")
            except Error as e:
                self.get_logger().error("Database error")

        elif self.from_gui_data == "out_signal":
            print("receive out signal")
            self.park_num_publisher.publish(self.park_num_msg)
            self.flag_num = False

        elif self.from_gui_data == "cancled":
            print("receive cancle signal")
            self.flag_num = False


    def __del__(self):
        if self.connection and self.connection.is_connected():
            self.cursor.close()
            self.connection.close()
            print("MySQL connection is closed")

def main(args=None):
    rp.init(args=args)
    take_db_data = Take_DB_Data()
    
    rp.spin(take_db_data)
    take_db_data.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
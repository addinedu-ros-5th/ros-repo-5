import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import pymysql
from datetime import datetime
from DB_SET import DB_SET

class CarInfoSubscriber(Node):
    def __init__(self):
        super().__init__('car_subscriber')
        self.in_car_subscription = self.create_subscription(
            String,
            'in_car',
            self.in_car_callback,
            10
        )
        self.out_car_subscription = self.create_subscription(
            String,
            'out_car',
            self.out_car_callback,
            10
        )
        self.db_connection = self.connect_to_db()

    def connect_to_db(self):
        return pymysql.connect(
            host=DB_SET['HOST'],
            user=DB_SET['USER'],
            password=DB_SET['PASSWORD'],
            database=DB_SET['DATABASE']
        )

    def in_car_callback(self, msg):
        car_info = json.loads(msg.data)
        self.get_logger().info(f'IN_CAR - CAR_NUM: {car_info["car_num"]}, TIME: {car_info["time"]}')
        self.update_car_info(car_info["car_num"], car_info["time"])

    def out_car_callback(self, msg):
        car_info = json.loads(msg.data)
        self.get_logger().info(f'OUT_CAR - CAR_NUM: {car_info["car_num"]}, TIME: {car_info["time"]}')
        self.update_car_log(car_info["car_num"], car_info["time"])

    def update_car_info(self, car_num, time):
        try:
            with self.db_connection.cursor() as cursor:
                timestamp = datetime.strptime(time, '%Y-%m-%d|%H:%M:%S')
                
                # Check if there's an existing entry in CAR_LOG without an out_time
                cursor.execute("SELECT id, in_time FROM CAR_LOG WHERE car_num = %s AND out_time IS NULL", (car_num,))
                result = cursor.fetchone()
                
                if result:
                    # Update existing entry in CAR_LOG
                    log_id, in_time = result
                    cursor.execute("UPDATE CAR_LOG SET out_time = %s WHERE id = %s", (timestamp, log_id))
                else:
                    # Insert new entry into CAR_LOG
                    cursor.execute("INSERT INTO CAR_LOG (car_num, in_time) VALUES (%s, %s)", (car_num, timestamp))
                
                # Always update or insert into CAR_INFO
                cursor.execute("REPLACE INTO CAR_INFO (CAR_NUM, IN_TIME, PK_NUM) VALUES (%s, %s, %s)", 
                               (car_num, timestamp, "I1"))
                
            self.db_connection.commit()
            self.get_logger().info(f"Database updated for IN_CAR {car_num}")
        except Exception as e:
            self.get_logger().error(f"Database error: {e}")

    def update_car_log(self, car_num, out_time):
        try:
            with self.db_connection.cursor() as cursor:
                out_timestamp = datetime.strptime(out_time, '%Y-%m-%d|%H:%M:%S')
                
                # Update CAR_LOG
                cursor.execute("UPDATE CAR_LOG SET out_time = %s WHERE car_num = %s AND out_time IS NULL", 
                               (out_timestamp, car_num))
                
                if cursor.rowcount == 0:
                    self.get_logger().warning(f"No open entry found in CAR_LOG for {car_num}")
                else:
                    # Remove from CAR_INFO
                    cursor.execute("DELETE FROM CAR_INFO WHERE CAR_NUM = %s", (car_num,))
                
            self.db_connection.commit()
            self.get_logger().info(f"Database updated for OUT_CAR {car_num}")
        except Exception as e:
            self.get_logger().error(f"Database error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CarInfoSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.db_connection.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
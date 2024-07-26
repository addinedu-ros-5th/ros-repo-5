import mysql.connector
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from DB_SET import DB_SET

class PlatePublisher(Node):
    def __init__(self):
        super().__init__('plate_publisher')
        self.plate_publisher = self.create_publisher(String, 'plate_topic', 10)
        self.in_time_publisher = self.create_publisher(String, 'in_time_topic', 10)
        self.out_time_publisher = self.create_publisher(String, 'out_time_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.data = self.fetch_data_from_db()
        self.data_iter = iter(self.data.items())

    def fetch_data_from_db(self):
        connection = mysql.connector.connect(
            host=DB_SET["HOST"],
            database=DB_SET["DATABASE"],
            user=DB_SET["USER"],
            password=DB_SET["PASSWORD"]
        )
        cursor = connection.cursor()
        cursor.execute("SELECT CAR_NUM, IN_TIME, OUT_TIME FROM CAR_INFO")
        rows = cursor.fetchall()
        data = {}
        for row in rows:
            car_num = row[0]
            in_time = row[1].strftime('%Y-%m-%d|%H:%M:%S') if row[1] is not None else 'N/A'
            out_time = row[2].strftime('%Y-%m-%d|%H:%M:%S') if row[2] is not None else 'N/A'
            data[car_num] = {'in_time': in_time, 'out_time': out_time}
        cursor.close()
        connection.close()
        return data

    def publish_data(self):
        try:
            plate, timestamps = next(self.data_iter)
            plate_msg = String()
            plate_msg.data = plate
            self.plate_publisher.publish(plate_msg)
            self.get_logger().info(f'Published plate: {plate_msg.data}')

            in_time_msg = String()
            in_time_msg.data = timestamps['in_time']
            self.in_time_publisher.publish(in_time_msg)
            self.get_logger().info(f'Published IN_TIME: {in_time_msg.data}')

            if timestamps['out_time'] != 'N/A':
                out_time_msg = String()
                out_time_msg.data = timestamps['out_time']
                self.out_time_publisher.publish(out_time_msg)
                self.get_logger().info(f'Published OUT_TIME: {out_time_msg.data}')
        except StopIteration:
            self.get_logger().info('All data published.')
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = PlatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import csv
from datetime import datetime, timedelta
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CarPublisher(Node):
    def __init__(self):
        super().__init__('car_publisher')
        self.in_car_publisher = self.create_publisher(String, 'in_car', 10)
        self.out_car_publisher = self.create_publisher(String, 'out_car', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.data = self.process_csv_data()
        self.data_iter = iter(self.data)

    def process_csv_data(self):
        file_path = "AI/car_plate_detection/data/detected_license_plates.csv"
        car_data = {}
        processed_data = []

        with open(file_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                plate = row['License Plate Text']
                timestamp = datetime.strptime(row['Capture Time'], '%Y-%m-%d %H:%M:%S')
                
                if plate not in car_data:
                    car_data[plate] = {'last_time': timestamp, 'in_car': True}
                    processed_data.append({'car_num': plate, 'time': timestamp, 'type': 'in_car'})
                else:
                    if timestamp - car_data[plate]['last_time'] >= timedelta(minutes=2):
                        if car_data[plate]['in_car']:
                            processed_data.append({'car_num': plate, 'time': timestamp, 'type': 'out_car'})
                            car_data[plate]['in_car'] = False
                        else:
                            processed_data.append({'car_num': plate, 'time': timestamp, 'type': 'in_car'})
                            car_data[plate]['in_car'] = True
                    car_data[plate]['last_time'] = timestamp

        return sorted(processed_data, key=lambda x: x['time'])

    def publish_data(self):
        try:
            car_data = next(self.data_iter)
            message = json.dumps({
                'car_num': car_data['car_num'],
                'time': car_data['time'].strftime('%Y-%m-%d|%H:%M:%S')
            })
            
            msg = String()
            msg.data = message
            
            if car_data['type'] == 'in_car':
                self.in_car_publisher.publish(msg)
                self.get_logger().info(f'Published in_car: {message}')
            else:
                self.out_car_publisher.publish(msg)
                self.get_logger().info(f'Published out_car: {message}')
                
        except StopIteration:
            self.get_logger().info('All data published.')
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = CarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
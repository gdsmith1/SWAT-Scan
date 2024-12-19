import cv2
import os
import zipfile
import paramiko
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Constants
DESTINATION_IP = '192.168.169.178'
DESTINATION_PORT = 22
USERNAME = 'robot7'
PASSWORD = 'wildcat'
REMOTE_PATH = '/home/robot7/snapshots.zip'
IMAGE_NAMES = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
SNAPSHOT_DIR = os.path.expanduser('~/admin/SWAT-Scan/temp-snapshots')
ZIP_FILENAME = 'snapshots.zip'

# Create directory if it doesn't exist
if not os.path.exists(SNAPSHOT_DIR):
    os.makedirs(SNAPSHOT_DIR)

webcam = cv2.VideoCapture(0)

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.vel_msg = Twist()
        
    def rotate_45_degrees(self):
        angular_speed = 0.7854
        rotation_duration = 1

        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = angular_speed

        self.publisher.publish(self.vel_msg)
        self.get_logger().info("Rotating 45 degrees")

        time.sleep(rotation_duration)

        self.vel_msg.angular.z = 0.0
        self.publisher.publish(self.vel_msg)
        self.get_logger().info("Stopping rotation")

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = TurtleBot3Controller()

    try:
        for index in range(8):
            turtlebot3_controller.rotate_45_degrees()
            time.sleep(4) 

            try:
                check, frame = webcam.read()
                print(check)  # prints true as long as the webcam is running
                print(frame)  # prints matrix values of each frame
                cv2.imshow("Capturing", frame)
                key = cv2.waitKey(1)

                filename = f'{SNAPSHOT_DIR}/{IMAGE_NAMES[index]}.jpg'
                cv2.imwrite(filename, frame)
                print(f"Image {IMAGE_NAMES[index]} saved!")

                if key == ord('q'):
                    print("Turning off camera.")
                    webcam.release()
                    print("Camera off.")
                    print("Program ended.")
                    cv2.destroyAllWindows()
                    exit()

            except KeyboardInterrupt:
                print("Turning off camera.")
                webcam.release()
                print("Camera off.")
                print("Program ended.")
                cv2.destroyAllWindows()
                break

        print("Finished capturing images.")
        webcam.release()
        cv2.destroyAllWindows()

        # Package images into a zip file
        print("Packaging images into a zip file...")
        with zipfile.ZipFile(ZIP_FILENAME, 'w') as zipf:
            for root, dirs, files in os.walk(SNAPSHOT_DIR):
                print(f"Found files: {files} in {root}")
                for file in files:
                    file_path = os.path.join(root, file)
                    zipf.write(file_path, os.path.relpath(file_path, SNAPSHOT_DIR))
                    print(f"Added {file_path} to zip file.")

        # Verify the zip file was created
        if not os.path.exists(ZIP_FILENAME):
            print(f"Error: {ZIP_FILENAME} was not created.")
        else:
            print(f"{ZIP_FILENAME} created successfully.")

        # Send the zip file to another machine using SFTP
        try:
            print("Sending the zip file to the destination machine...")
            transport = paramiko.Transport((DESTINATION_IP, DESTINATION_PORT))
            transport.connect(username=USERNAME, password=PASSWORD)
            sftp = paramiko.SFTPClient.from_transport(transport)
            sftp.put(ZIP_FILENAME, REMOTE_PATH)
            sftp.close()
            transport.close()
            print("Images sent to the destination machine.")
        except Exception as e:
            print(f"Failed to send images: {e}")

    finally:
        turtlebot3_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import cv2
import os
import time
import zipfile
import paramiko

# Constants
DESTINATION_IP = '10.243.254.139'
DESTINATION_PORT = 22
USERNAME = 'csc2'
PASSWORD = 'wildcat'
REMOTE_PATH = '/path/to/destination/snapshots.zip'
IMAGE_NAMES = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
SNAPSHOT_DIR = '~/temp-snapshots'
ZIP_FILENAME = 'snapshots.zip'

# Create directory if it doesn't exist
if not os.path.exists(SNAPSHOT_DIR):
    os.makedirs(SNAPSHOT_DIR)

webcam = cv2.VideoCapture(0)
start_time = time.time()
index = 0

while True:
    try:
        check, frame = webcam.read()
        print(check)  # prints true as long as the webcam is running
        print(frame)  # prints matrix values of each frame
        cv2.imshow("Capturing", frame)
        key = cv2.waitKey(1)

        # Take a picture every 2 seconds for 16 seconds
        if time.time() - start_time >= 2 * index and index < 8:
            filename = f'{SNAPSHOT_DIR}/{IMAGE_NAMES[index]}.jpg'
            cv2.imwrite(filename, frame)
            print(f"Image {IMAGE_NAMES[index]} saved!")
            index += 1

        # Exit after 16 seconds
        if time.time() - start_time >= 4: #16:
            print("Finished capturing images.")
            webcam.release()
            cv2.destroyAllWindows()
            break

        # Allow user to quit early
        if key == ord('q'):
            print("Turning off camera.")
            webcam.release()
            print("Camera off.")
            print("Program ended.")
            cv2.destroyAllWindows()
            break

    except KeyboardInterrupt:
        print("Turning off camera.")
        webcam.release()
        print("Camera off.")
        print("Program ended.")
        cv2.destroyAllWindows()
        break

# Package images into a zip file
print("Packaging images into a zip file...")
with zipfile.ZipFile(ZIP_FILENAME, 'w') as zipf:
    for image_name in IMAGE_NAMES:
        image_path = f'{SNAPSHOT_DIR}/{image_name}.jpg'
        if os.path.exists(image_path):
            zipf.write(image_path)
        else:
            print(f"Warning: {image_path} does not exist and will not be included in the zip file.")

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
import cv2
import os
import time

# Create directory if it doesn't exist
if not os.path.exists('temp-snapshots'):
    os.makedirs('temp-snapshots')

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

        # Take a picture every 2 seconds for 10 seconds
        if time.time() - start_time >= 2 * index and index < 5:
            filename = f'temp-snapshots/snapshot_{index}.jpg'
            cv2.imwrite(filename, frame)
            print(f"Image {index} saved!")
            index += 1

        # Exit after 10 seconds
        if time.time() - start_time >= 10:
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
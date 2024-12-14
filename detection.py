import cv2
import numpy as np
import os
import zipfile
import shutil

# Calculate the percentage of red pixels in the image
def calculate_red_percentage(image):
    # Define the red color range in BGR
    lower_red = np.array([0, 0, 100])
    upper_red = np.array([50, 50, 255])

    # Create a mask for red pixels
    mask = cv2.inRange(image, lower_red, upper_red)

    # Calculate the percentage of red pixels
    red_pixels = cv2.countNonZero(mask)
    total_pixels = image.shape[0] * image.shape[1]
    red_percentage = (red_pixels / total_pixels) * 100

    return red_percentage

# Calculate the percentage of green pixels in the image
def calculate_green_percentage(image):
    # Define the green color range in BGR
    lower_green = np.array([0, 50, 0])
    upper_green = np.array([100, 255, 100])

    # Create a mask for green pixels
    mask = cv2.inRange(image, lower_green, upper_green)

    # Calculate the percentage of green pixels
    green_pixels = cv2.countNonZero(mask)
    total_pixels = image.shape[0] * image.shape[1]
    green_percentage = (green_pixels / total_pixels) * 100

    return green_percentage

# Calculate the percentage of blue pixels in the image
def calculate_blue_percentage(image):
    # Define the blue color range in BGR
    lower_blue = np.array([100, 0, 0])
    upper_blue = np.array([255, 100, 100])

    # Create a mask for blue pixels
    mask = cv2.inRange(image, lower_blue, upper_blue)

    # Calculate the percentage of blue pixels
    blue_pixels = cv2.countNonZero(mask)
    total_pixels = image.shape[0] * image.shape[1]
    blue_percentage = (blue_pixels / total_pixels) * 100

    return blue_percentage

# Load the pre-trained Haar Cascade Classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def main():
    # Get the zip file path from the user
    zip_file_path = input("Enter the path to the zip file: ")
    extract_to_dir = "extracted_images"

    # Unzip the folder
    with zipfile.ZipFile(zip_file_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to_dir)

    try:
        # Find the name of the unzipped folder
        unzipped_folder = os.path.join(extract_to_dir, os.listdir(extract_to_dir)[0])

        # Loop through all files in the unzipped folder
        red_percentages = []
        green_percentages = []
        blue_percentages = []
        image_names = []
        for file_name in os.listdir(unzipped_folder):
            image_path = os.path.join(unzipped_folder, file_name)
            img = cv2.imread(image_path)
            if img is None:
                print(f"Skipping non-image file or invalid image: {file_name}")
                continue

            # Calculate the percentage of red pixels
            red_percentage = calculate_red_percentage(img)
            red_percentages.append(red_percentage)

            # Calculate the percentage of green pixels
            green_percentage = calculate_green_percentage(img)
            green_percentages.append(green_percentage)

            # Calculate the percentage of blue pixels
            blue_percentage = calculate_blue_percentage(img)
            blue_percentages.append(blue_percentage)

            image_names.append(file_name)

            # Convert the image to grayscale for face detection
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Detect faces in the image
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=7, minSize=(30, 30))

            # Print the number of faces found only if there are more than zero faces
            if len(faces) > 0:
                print(f"Image '{file_name}' has {len(faces)} faces detected.")

        # Find the image with the most red
        if red_percentages and max(red_percentages) > 0:
            max_red_index = np.argmax(red_percentages)
            print(f"Image with the most red: {image_names[max_red_index]}")
            print(f"Percentage of red: {red_percentages[max_red_index]:.2f}%")
        else:
            print("No red found.")

        # Find the image with the most green
        if green_percentages and max(green_percentages) > 0:
            max_green_index = np.argmax(green_percentages)
            print(f"Image with the most green: {image_names[max_green_index]}")
            print(f"Percentage of green: {green_percentages[max_green_index]:.2f}%")
        else:
            print("No green found.")

        # Find the image with the most blue
        if blue_percentages and max(blue_percentages) > 0:
            max_blue_index = np.argmax(blue_percentages)
            print(f"Image with the most blue: {image_names[max_blue_index]}")
            print(f"Percentage of blue: {blue_percentages[max_blue_index]:.2f}%")
        else:
            print("No blue found.")

    finally:
        # Cleanup: delete the extracted images folder
        if os.path.exists(extract_to_dir):
            shutil.rmtree(extract_to_dir)

if __name__ == "__main__":
    main()

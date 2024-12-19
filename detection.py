import cv2
import numpy as np
import os
import zipfile
import shutil
import time

# Calculate the percentage of red pixels in the image
def calculate_red_percentage(image):
    lower_red = np.array([0, 0, 100])
    upper_red = np.array([50, 50, 255])
    mask = cv2.inRange(image, lower_red, upper_red)
    red_pixels = cv2.countNonZero(mask)
    total_pixels = image.shape[0] * image.shape[1]
    red_percentage = (red_pixels / total_pixels) * 100
    return red_percentage

def calculate_green_percentage(image):
    lower_green = np.array([0, 50, 0])
    upper_green = np.array([100, 255, 100])
    mask = cv2.inRange(image, lower_green, upper_green)
    green_pixels = cv2.countNonZero(mask)
    total_pixels = image.shape[0] * image.shape[1]
    green_percentage = (green_pixels / total_pixels) * 100
    return green_percentage

def calculate_blue_percentage(image):
    lower_blue = np.array([100, 0, 0])
    upper_blue = np.array([255, 100, 100])
    mask = cv2.inRange(image, lower_blue, upper_blue)
    blue_pixels = cv2.countNonZero(mask)
    total_pixels = image.shape[0] * image.shape[1]
    blue_percentage = (blue_pixels / total_pixels) * 100
    return blue_percentage

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def wait_for_zip_file(zip_file_path):
    while not os.path.exists(zip_file_path):
        time.sleep(1)
    print(f"Found {zip_file_path}! Proceeding with the program.")

def main():
    zip_file_path = "snapshots.zip"
    extract_to_dir = "extracted_images"

    wait_for_zip_file(zip_file_path)

    with zipfile.ZipFile(zip_file_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to_dir)

    try:
        unzipped_folder = os.path.join(extract_to_dir)

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

            red_percentage = calculate_red_percentage(img)
            red_percentages.append(red_percentage)

            green_percentage = calculate_green_percentage(img)
            green_percentages.append(green_percentage)

            blue_percentage = calculate_blue_percentage(img)
            blue_percentages.append(blue_percentage)

            image_names.append(file_name)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=7, minSize=(30, 30))

            if len(faces) > 0:
                print(f"Image '{file_name}' has {len(faces)} faces detected.")

        if red_percentages and max(red_percentages) > 0:
            max_red_index = np.argmax(red_percentages)
            print(f"Image with the most red: {image_names[max_red_index]}")
            print(f"Percentage of red: {red_percentages[max_red_index]:.2f}%")
        else:
            print("No red found.")

        if green_percentages and max(green_percentages) > 0:
            max_green_index = np.argmax(green_percentages)
            print(f"Image with the most green: {image_names[max_green_index]}")
            print(f"Percentage of green: {green_percentages[max_green_index]:.2f}%")
        else:
            print("No green found.")

        if blue_percentages and max(blue_percentages) > 0:
            max_blue_index = np.argmax(blue_percentages)
            print(f"Image with the most blue: {image_names[max_blue_index]}")
            print(f"Percentage of blue: {blue_percentages[max_blue_index]:.2f}%")
        else:
            print("No blue found.")

    finally:
        if os.path.exists(extract_to_dir):
            shutil.rmtree(extract_to_dir)

if __name__ == "__main__":
    main()

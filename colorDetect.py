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
            image_names.append(file_name)

        # Find the image with the most red
        if red_percentages:
            max_red_index = np.argmax(red_percentages)
            print(f"Image with the most red: {image_names[max_red_index]}")
            print(f"Percentage of red: {red_percentages[max_red_index]:.2f}%")
        else:
            print("No images were found or processed.")
    finally:
        # Cleanup: delete the extracted images folder
            if os.path.exists(extract_to_dir):
                shutil.rmtree(extract_to_dir)

if __name__ == "__main__":
    main()
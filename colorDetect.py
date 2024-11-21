import numpy as np
import cv2
from sklearn.cluster import KMeans
from collections import Counter
import matplotlib.pyplot as plt

# Finds the most common color
def palette_most_common(k_cluster):
    most_common_cluster = Counter(k_cluster.labels_).most_common(1)[0][0]
    most_common_color = k_cluster.cluster_centers_[most_common_cluster]
    return np.full((50, 300, 3), most_common_color, dtype=np.uint8)

# Show the most common color (using matplotlib)
def show_palette(palette):
    plt.figure(figsize=(8, 2))
    plt.axis("off")
    plt.imshow(cv2.cvtColor(palette, cv2.COLOR_BGR2RGB))
    plt.title("Most Common Color")
    plt.show()

def main():
    image_path = input("Enter the path to the image file: ")

    # Load image
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Image not found. Please check the path.")
        return

    # Resize for speedup
    img_resized = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))

    # Convert image to a 2D array of pixels
    pixels = img_resized.reshape(-1, 3)

    # Apply KMeans clustering (using 5 clusters)
    n_clusters = 5
    clt = KMeans(n_clusters=n_clusters, random_state=0)
    clt.fit(pixels)

    # Find the most common color
    palette = palette_most_common(clt)

    # Display the color
    show_palette(palette)

if __name__ == "__main__":
    main()
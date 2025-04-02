import json
import matplotlib.pyplot as plt
import numpy as np

# Load JSON data
with open("notes/poses.json", "r") as file:
    data = json.load(file)

img = plt.imread("notes/field.png")

# Extract coordinates and keys
coordinates = {key: (value["x"]["val"], value["y"]["val"]) for key, value in data.items()}
print(coordinates)
# Define field dimensions
field_width = 17.37  # meters
field_height = 8.22 # meters

# Create figure and axis
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(0, field_width)
ax.set_ylim(0, field_height)
ax.set_title("Pose Visualization")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

# Plot the points and labels
ax.imshow(img, extent=[0, 17.37, 0, 8.22])
for key, (x, y) in coordinates.items():
    ax.scatter(x, y, color="yellow", marker="o")
    ax.text(x , y, key, fontsize=8, color="purple")

# Show grid
ax.grid(True, linestyle="--", alpha=0.5)


# Display the plot
plt.show()

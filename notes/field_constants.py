import json
import matplotlib.pyplot as plt

# Load the JSON data
file_path = "field_constants.json"
with open(file_path, "r") as file:
    data = json.load(file)

# Extract field dimensions
field_length = data.get("fieldLength", 10)  # Default if missing
field_width = data.get("fieldWidth", 5)  # Default if missing

# Function to extract unique coordinates from nested dicts
def extract_unique_coordinates(prefix, obj, locations, seen_points):
    if isinstance(obj, dict):
        if "x" in obj and "y" in obj:
            point = (obj["x"], obj["y"])  # Create a tuple of coordinates
            if point not in seen_points:
                seen_points.add(point)
                locations[prefix] = obj  # Store only unique points
        else:
            for key, value in obj.items():
                extract_unique_coordinates(f"{prefix} - {key}", value, locations, seen_points)
    elif isinstance(obj, list):
        for i, item in enumerate(obj):
            extract_unique_coordinates(f"{prefix} {i+1}", item, locations, seen_points)

# Extract unique locations
locations = {}
seen_points = set()  # Track seen coordinates to avoid duplicates
for key, value in data.items():
    extract_unique_coordinates(key, value, locations, seen_points)

# Setup plot
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(0, field_length)
ax.set_ylim(0, field_width)
ax.set_xlabel("Field Length (m)")
ax.set_ylabel("Field Width (m)")
ax.set_title("Visualization of Unique Field Locations")

# Define colors based on categories
colors = {
    "Staging": "blue",
    "Reef": "red",
    "Loader": "green",
    "Barge": "purple",
}

# Label offset settings
x_offset = 0.15
y_offset = 0.15

# Plot unique locations
for name, pos in locations.items():
    category = next((key for key in colors.keys() if key in name), "Other")  # Find matching category
    color = colors.get(category, "gray")  # Default color

    # Plot point
    ax.scatter(pos["x"], pos["y"], color=color, label=name if name not in ax.get_legend_handles_labels()[1] else "")

    # Add label with background
    ax.text(pos["x"] + x_offset, pos["y"] + y_offset, name, fontsize=6, 
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none', boxstyle="round,pad=0.2"))

# Add legend (remove duplicates)
handles, labels = ax.get_legend_handles_labels()
unique_labels = list(dict.fromkeys(labels))  # Remove duplicate labels
ax.legend(handles[:len(unique_labels)], unique_labels[:len(unique_labels)], fontsize=7)

plt.grid(True)
plt.show()

# Debugging: Print extracted unique locations
print("Extracted Unique Locations:")
for name, pos in locations.items():
    print(f"{name}: {pos}")

import numpy as np
import matplotlib.pyplot as plt
import os


# ============================================================
# 1. Load occupancy map
# ============================================================

script_dir = os.path.dirname(os.path.abspath(__file__))

occupancy_file = os.path.join(
    script_dir,
    "data",
    "sydney_local_occupancy.npy"
)

occupancy = np.load(occupancy_file)


# ============================================================
# 2. Map parameters
#    Must be the same as sydney_occupancy.py
# ============================================================

xmin = -700.0
xmax = -300.0

ymin = 100.0
ymax = 400.0

resolution = 2.0


# ============================================================
# 3. Curved path settings
# ============================================================

# Center of the circular arc
center = np.array([-500.0, 200.0])

# Radius [m]
radius = 50.0

# Start and end angle [degree]
start_angle_deg = 180.0
end_angle_deg = 90.0

# Distance between output waypoints [m]
waypoint_spacing = 10.0


# ============================================================
# 4. Calculate arc length and number of points
# ============================================================

start_angle = np.deg2rad(start_angle_deg)
end_angle = np.deg2rad(end_angle_deg)

angle_range = abs(end_angle - start_angle)

arc_length = radius * angle_range

num_points = int(np.ceil(arc_length / waypoint_spacing)) + 1

angles = np.linspace(
    start_angle,
    end_angle,
    num_points
)


# ============================================================
# 5. Generate curved path
# ============================================================

x = center[0] + radius * np.cos(angles)
y = center[1] + radius * np.sin(angles)

path = np.column_stack((x, y))


# ============================================================
# 6. Start and goal
# ============================================================

start = path[0]
goal = path[-1]


# ============================================================
# 7. Plot occupancy map
# ============================================================

plt.figure(figsize=(12, 8))

plt.imshow(
    occupancy,
    origin="lower",
    extent=[xmin, xmax, ymin, ymax],
    cmap="gray_r",
    interpolation="nearest"
)


# ============================================================
# 8. Overlay curved path
# ============================================================

plt.plot(
    path[:, 0],
    path[:, 1],
    "o-",
    linewidth=2,
    markersize=5,
    label="Curved path"
)

plt.scatter(
    start[0],
    start[1],
    s=100,
    marker="o",
    label="Start"
)

plt.scatter(
    goal[0],
    goal[1],
    s=100,
    marker="x",
    label="Goal"
)

plt.scatter(
    center[0],
    center[1],
    s=60,
    marker="+",
    label="Arc center"
)


# ============================================================
# 9. Plot settings
# ============================================================

plt.xlabel("World X [m]")
plt.ylabel("World Y [m]")

plt.title("Sydney Regatta - Curved Reference Path")

plt.axis("equal")
plt.legend()
plt.grid(True)


# ============================================================
# 10. Output folder
# ============================================================

output_dir = os.path.join(
    script_dir,
    "output"
)

os.makedirs(
    output_dir,
    exist_ok=True
)


# ============================================================
# 11. Save files
# ============================================================

csv_file = os.path.join(
    output_dir,
    "sydney_curve_path.csv"
)

npy_file = os.path.join(
    output_dir,
    "sydney_curve_path.npy"
)

png_file = os.path.join(
    output_dir,
    "sydney_curve_path.png"
)

np.savetxt(
    csv_file,
    path,
    delimiter=",",
    header="x,y",
    comments=""
)

np.save(
    npy_file,
    path
)

plt.savefig(
    png_file,
    dpi=150,
    bbox_inches="tight"
)

plt.show()


# ============================================================
# 12. Print information
# ============================================================

print("Curved path generated successfully.")

print(f"Center: {center}")
print(f"Radius: {radius:.2f} m")

print(
    f"Angle: "
    f"{start_angle_deg:.1f} deg "
    f"-> {end_angle_deg:.1f} deg"
)

print(f"Arc length: {arc_length:.2f} m")
print(f"Number of waypoints: {len(path)}")

print("\nStart:")
print(
    f"x = {start[0]:.2f}, "
    f"y = {start[1]:.2f}"
)

print("\nGoal:")
print(
    f"x = {goal[0]:.2f}, "
    f"y = {goal[1]:.2f}"
)

print("\nWaypoints:")

for i, point in enumerate(path):
    print(
        f"{i:02d}: "
        f"x = {point[0]:.2f}, "
        f"y = {point[1]:.2f}"
    )

print("\nFiles saved:")
print(csv_file)
print(npy_file)
print(png_file)
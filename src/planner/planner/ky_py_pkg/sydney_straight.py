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
#    MUST be identical to sydney_occupancy.py
# ============================================================

xmin = -700.0
xmax = -300.0

ymin = 100.0
ymax = 400.0

resolution = 2.0


# ============================================================
# 3. Straight path settings
# ============================================================

start = np.array([-530.0, 170.0])
goal = np.array([-450.0, 250.0])

waypoint_spacing = 10.0


# ============================================================
# 4. Generate straight-line waypoints
# ============================================================

direction = goal - start
distance = np.linalg.norm(direction)

num_segments = int(np.floor(distance / waypoint_spacing))

path = []

for i in range(num_segments + 1):
    d = min(i * waypoint_spacing, distance)

    if distance > 0:
        point = start + direction / distance * d
    else:
        point = start.copy()

    path.append(point)

# Make sure goal is included
if np.linalg.norm(path[-1] - goal) > 1e-6:
    path.append(goal)

path = np.array(path)


# ============================================================
# 5. Plot occupancy map
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
# 6. Overlay straight path
# ============================================================

plt.plot(
    path[:, 0],
    path[:, 1],
    "o-",
    linewidth=2,
    markersize=5,
    label="Straight path"
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


# ============================================================
# 7. Plot settings
# ============================================================

plt.xlabel("World X [m]")
plt.ylabel("World Y [m]")

plt.title("Sydney Regatta - Straight Path on Occupancy Map")

plt.axis("equal")
plt.legend()
plt.grid(True)


# ============================================================
# 8. Save results
# ============================================================

output_dir = os.path.join(script_dir, "output")
os.makedirs(output_dir, exist_ok=True)

csv_file = os.path.join(
    output_dir,
    "sydney_straight_path.csv"
)

npy_file = os.path.join(
    output_dir,
    "sydney_straight_path.npy"
)

png_file = os.path.join(
    output_dir,
    "sydney_straight_path.png"
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
# 9. Print result
# ============================================================

print("Straight path generated.")
print(f"Start: {start}")
print(f"Goal: {goal}")
print(f"Distance: {distance:.2f} m")
print(f"Waypoints: {len(path)}")

for i, point in enumerate(path):
    print(
        f"{i:02d}: "
        f"x = {point[0]:.2f}, "
        f"y = {point[1]:.2f}"
    )
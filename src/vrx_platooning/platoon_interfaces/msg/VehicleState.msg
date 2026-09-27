std_msgs/Header header

# World NED position
# x = North [m]
# y = East [m]
float64 x
float64 y

# World NED velocity
# vx = North velocity [m/s]
# vy = East velocity [m/s]
float64 vx
float64 vy

float64 speed

# Direction of motion in world NED.
# 0 = North, +pi/2 = East.
float64 course_angle

# Vehicle bow heading in world NED.
# 0 = North, +pi/2 = East.
float64 body_yaw

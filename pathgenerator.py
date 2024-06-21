import matplotlib.pyplot as plt
import numpy as np

def bezier_curve(x_pts: list, y_pts: list):
    x0, x1, x2, x3 = x_pts
    y0, y1, y2, y3 = y_pts
    t = np.linspace(0, 1, 100)
    x = (1-t)**3*x0 + 3*t*(1-t)**2*x1 + 3*t**2*(1-t)*x2 + t**3*x3
    y = (1-t)**3*y0 + 3*t*(1-t)**2*y1 + 3*t**2*(1-t)*y2 + t**3*y3
    return x, y

x_control_pts = [-5.2, 7, 0.01, 3.74]
y_control_pts = [-2.95, -8.63, -2.1, 0]

x_coords, y_coords = bezier_curve(x_control_pts, y_control_pts)
img = plt.imread('over_under_field.jpeg')
fig, ax = plt.subplots()
ax.imshow(img, extent=[-6, 6, -6, 6])
ax.plot(x_coords, y_coords, '--', color='springgreen')
# ax.scatter(x_control_pts, y_control_pts, s=10, color='firebrick')
plt.show()
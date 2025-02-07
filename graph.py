import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.colors as colors
import numpy as np

plt.style.use('_mpl-gallery')
mpl.rcParams['axes3d.mouserotationstyle'] = 'azel'

path_1 = {
    "center": [0, 0],
    "radius": 15,
    "start_angle": 0,
    "end_angle": 90,
    "start_height": 25,
    "end_height": 15
}
path_2 = {
    "center": [10, 10],
    "radius": 5,
    "start_angle": -45,
    "end_angle": 225,
    "start_height": 18,
    "end_height": 30
}
path_3 = {
    "center": [10, -10],
    "radius": 15,
    "start_angle": -15,
    "end_angle": 15,
    "start_height": 60,
    "end_height": 10
}

obstacle_1 = {
    "polygon": [[10, 10], [10, 0], [0, 10]],
    "height": 30
}
obstacle_2 = {
    "polygon": [[10, 0], [10, 10], [20, 10], [20, 0]],
    "height": 20
}
obstacle_3 = {
    "polygon": [[0, 10], [0, 20], [10, 20], [10, 10]],
    "height": 20
}

def plot_path(ax, path):
    center = path["center"]
    radius = path["radius"]
    start_angle = path["start_angle"]
    end_angle = path["end_angle"]
    start_height = path["start_height"]
    end_height = path["end_height"]
    n = 100
    theta = np.linspace(np.radians(start_angle), np.radians(end_angle), n)
    x = center[0] + radius * np.sin(theta)
    y = center[1] + radius * np.cos(theta)
    z = np.linspace(start_height, end_height, n)
    ax.plot(x, y, z)

def plot_obstacle(ax, obstacle):
    polygon = obstacle["polygon"]
    height = obstacle["height"]
    cap = [[point[0], point[1], height] for point in polygon]
    polygon += [polygon[0]]
    print('polygon', polygon)
    sides = []
    for i in range(len(polygon) - 1):
        p1 = polygon[i]
        print('p1', p1)
        p2 = polygon[i + 1]
        print('p2', p2)
        side = [p1 + [0], p2 + [0], p2 + [height], p1 + [height]]
        sides += [side]
    sides += [cap]
    print('sides', sides)
    for side in sides:
        poly = Poly3DCollection([side])
        print('poly', poly)
        poly.set_color(colors.rgb2hex(np.random.rand(3)))
        poly.set_alpha(0.5)
        poly.set_edgecolor('k')
        ax.add_collection3d(poly)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
plot_path(ax, path_3)
plot_obstacle(ax, obstacle_2)
ax.set(xlabel='x', ylabel='y', zlabel='z')
#plt.savefig('path_1__obstacle_3.png', dpi=300)
plt.show()





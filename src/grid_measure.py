#! /home/ratatoskgs/rotor_control/.venv/bin/python
import sys,json,os,subprocess
import numpy as np
from utils import XY2vec,vec2XY
from matplotlib import pyplot as plt
from time import sleep


scew = lambda v: np.array([
    [    0, -v[2],  v[1]],
    [ v[2],     0, -v[0]],
    [-v[1],  v[0],     0]
])

R_v = lambda v,a: np.cos(a) * np.eye(3) + np.sin(a) * scew(v) + (1 - np.cos(a)) * np.outer(v, v)


deg = 1
grid_size = 7
measurement_time = 0.1

if len(sys.argv) == 1:
    print("Missing input arguments X and Y")
    print("[X] [Y] (degrees span) (grid size) (measure time)")
    inp = input("Use current position as center? [Y/n]")
    if inp == "" or inp.lower() == "y":
        cmd = f"{os.path.expanduser('~/')}rotor_control/build/rotor_control read 1"
        proc_set_angles = subprocess.run(cmd,shell=True,capture_output=True, text=True)
        if proc_set_angles.returncode != 0:
            exit()
        X_cen,Y_cen = list(map(float,proc_set_angles.stdout.split(",")))
    else:
        exit()
elif len(sys.argv) == 3:
    X_cen = float(sys.argv[1])
    Y_cen = float(sys.argv[2])
    print(f"Grid center: X: {X_cen} Y: {Y_cen}")
    print("No extra settings, defaulting to:")
    print(f"Degrees span +/-{deg} ")
    print(f"Grid size {grid_size}x{grid_size}")
    print(f"Per point measurement time {measurement_time}")
elif len(sys.argv) == 6:
    X_cen = float(sys.argv[1])
    Y_cen = float(sys.argv[2])
    deg = float(sys.argv[3])
    grid_size = int(sys.argv[4])
    measurement_time = float(sys.argv[5])
    print("Using values:")
    print(f"Grid center: X: {X_cen} Y: {Y_cen}")
    print(f"Degrees span +/-{deg} ")
    print(f"Grid size {grid_size}x{grid_size}")
    print(f"Per point measurement time {measurement_time}")
else:
    print("Only exactly 0, 2 or 5 arguments are accepted")
    print("[X] [Y] (degrees span) (grid size) (measure time)")
    exit()


# generate grid

i2g = lambda i,g,d: (i * 2/(g-1) - 1) * d
grid = np.array([[(i2g(i,grid_size,deg),i2g(j,grid_size,deg)) for j in range(grid_size)] for i in range(grid_size)])
# "Az El" format degrees
fname = "beacons_and_pointing_data.json"
with open(fname, "r") as f:
    data = json.load(f)

rf_offset = data["gs"]["off"]

vec = XY2vec(X_cen,Y_cen,rf_offset)

n_vec = np.cross(np.array([0,0,1]),vec)
n_vec /= np.linalg.norm(n_vec)

n_vec2 = (R_v(n_vec,-90 * np.pi/180) @ vec.transpose()).transpose()

R_grid = lambda a,b,n1,n2,v: (R_v(n1, a * np.pi/180) @ R_v(n2, b * np.pi/180) @ v.transpose()).transpose()

vec_grid = np.array([[R_grid(grid[i,j][0], grid[i,j][1], n_vec, n_vec2, vec) for j in range(grid_size)] for i in range(grid_size)])

def plot_vec_grid():
    range_ = range(2,178+2,2)
    surface_grid_size = len(list(range_))
    points = [[0 for _ in range(surface_grid_size)] for _ in range(surface_grid_size)]
    for i,X in enumerate(range_):
        for j,Y in enumerate(range_):
            points[i][j] = XY2vec(X,Y,rf_offset)
    points = np.array(points)

    fig, ax = plt.subplots(subplot_kw={"projection":"3d"})

    ax.plot_surface(points[:,:,0],points[:,:,1],points[:,:,2],linewidth=0,alpha=0.5)

    for i in range(grid_size):
        for j in range(grid_size):
            ax.scatter(*vec_grid[i,j],color="blue",s=2)
    plt.show()

# check if within [0,180]
for i in range(grid_size):
    for j in range(grid_size):
        X,Y = vec2XY(vec_grid[i,j],rf_offset)
        if not (0 < X < 180 and 0 < Y < 180):
            print("Grid outside of reachable region!")
            print("(i,j): ({i},{j})")
            print(f"X: {X} Y: {Y}")
            plot_vec_grid()
            print("Exiting")
            exit()

plot_vec_grid()


power_grid = [[None for j in range(grid_size)] for i in range(grid_size)]
dist2 = lambda X,Y,Xm,Ym: (X-Xm)**2+(Y-Ym)**2

# Test file input
import contextlib
with contextlib.suppress(FileNotFoundError):
    os.remove(f"{os.path.expanduser('~/')}rotator_measured")
    os.remove(f"{os.path.expanduser('~/')}power_sample.txt")
while True:
    try:
        with open(f"{os.path.expanduser('~/')}rotator_measured","r") as f: 
            X_measured,Y_measured = list(map(float, f.read().strip().split(" ")))
        with open(f"{os.path.expanduser('~/')}power_sample.txt","r") as f:
            power = float(f.read().strip())
        break
    except Exception as e:
        print(e)
        print("missing either angle or power input, sleeping for 1 second")
        sleep(1)
        continue

# goto grid start (rotor_control do-control)
X,Y = vec2XY(vec_grid[0,0],rf_offset)
print(f"Moving to grid start: X: {round(X,2)} Y: {round(Y,2)}")

for grid_index in range(grid_size**2):
    grid_i,grid_j = grid_index//grid_size,grid_index%grid_size
    X,Y = vec2XY(vec_grid[grid_i,grid_j],rf_offset)

    with open(os.path.expanduser("~/.rotator"), "w") as f:
        f.write(f"{round(X,2)} {round(Y,2)}")
    os.system(f"mv {os.path.expanduser('~/.rotator')} {os.path.expanduser('~/rotator')}")

    # 2 check if close enough/stopped
    while True:
        with open(f"{os.path.expanduser('~/')}rotator_measured","r") as f: 
            X_measured,Y_measured = list(map(float, f.read().strip().split(" ")))
        if dist2(X,Y,X_measured,Y_measured) > 0.05**2:
            sleep(0.05)
            continue
        break
    # 3 wait a bit (measure_time)
    sleep(measurement_time)
    # 4 read power file and save to output power grid
    with open(f"{os.path.expanduser('~/')}power_sample.txt","r") as f:
        power = float(f.read().strip())
    power_grid[grid_i][grid_j] = [X,Y,power]
    print(f"Got Power {power} at X: {round(X,2)} Y: {round(Y,2)} ")

# 5 save to grid
print(power_grid)
np.save(f"{os.path.expanduser('~/')}/rotor_control/power_measurements/power_grid_X_{str(X_cen).replace('.','_')}_Y_{str(Y_cen).replace('.','_')}",np.array(power_grid))
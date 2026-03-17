#! /home/ratatoskgs/rotor_control/.venv/bin/python
import numpy as np
import json
from utils import geo_to_ecef,vec2XY
import sys
import os

def main():
    # load ground station config
    fname = "beacons_and_pointing_data.json"
    with open(fname, "r") as f:
        data = json.load(f)

    gs_geo = [data["gs"]["lat"],data["gs"]["lon"],data["gs"]["alt"]]
    rf_offset = data["gs"]["off"]

    gs_R_E = np.load(os.path.expanduser("~/gs_rotation_calibration.npy"))

    # Calculate the vectors (Earth Centered Earth Fixed frame)
    v_gs = geo_to_ecef(*gs_geo)

    with open(os.path.expanduser("~/balloon_gnss.txt")) as f:
        baloon_gnss = list(map(float, f.read().strip().split(",")))
    lat_b,long_b,alt_b = baloon_gnss[0],baloon_gnss[1],baloon_gnss[2]
    v_b = geo_to_ecef(lat_b,long_b,alt_b)

    # get the pointing vector between the ground station and the balloon
    v_point_E = v_b - v_gs
    v_point_E = v_point_E/np.linalg.norm(v_point_E)

    # get the pointing vector in the ground station frame using the calibrated rotation matrix
    v_point_gs = gs_R_E @ v_point_E

    # calculate the X and Y from the vector in the ground station frame
    X,Y = vec2XY(v_point_gs, rf_offset)

    with open(os.path.expanduser("~/.rotator"), "w") as f:
        f.write(f"{round(X,2)} {round(Y,2)}")
    os.system(f"mv {os.path.expanduser('~/.rotator')} {os.path.expanduser('~/rotator')}")


if __name__ == "__main__":
    main()

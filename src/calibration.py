import numpy as np
import json
from utils import XY2vec,geo_to_ecef,best_fit_rotation_two_vectors
from matplotlib import pyplot as plt


def main():
    fname = "beacons_and_pointing_data.json"
    with open(fname, "r") as f:
        data = json.load(f)

    gs_geo = [data["gs"]["lat"],data["gs"]["lon"],data["gs"]["alt"]]
    rf_offset = data["gs"]["off"]

    beacon1_geo = [data["b1"]["lat"],data["b1"]["lon"],data["b1"]["alt"]]
    beacon2_geo = [data["b2"]["lat"],data["b2"]["lon"],data["b2"]["alt"]]
    b1_xy = data["b1_point"]
    b2_xy = data["b2_point"]

    # gs_geo = (55.36998744101496, 10.434779978287253, 23.46)
    # beacon1_geo = (55.3690405675022, 10.434861661343309, 23.46)
    # beacon2_geo = (55.36908437446848, 10.435963374896545, 23.46)

    print(gs_geo,beacon1_geo,beacon2_geo)

    v_gs = geo_to_ecef(*gs_geo)
    v_b1 = geo_to_ecef(*beacon1_geo)
    v_b2 = geo_to_ecef(*beacon2_geo)


    # pointing vectors from geo data:
    # v1_geo
    # v2_geo
    # pointing vectors from pointing data
    # v1_p
    # v2_p

    print(v_b1,v_b2,v_gs)
    #p = (x,y,z)
    v1_geo = v_b1 - v_gs
    v2_geo = v_b2 - v_gs
    v1_geo /= np.linalg.norm(v1_geo)
    v2_geo /= np.linalg.norm(v2_geo)

    v1_p = XY2vec(*b1_xy,rf_offset)
    v2_p = XY2vec(*b2_xy,rf_offset)

    a1 = np.arccos(np.dot(v1_geo,v2_geo)) * 180/np.pi

    a2 = np.arccos(np.dot(v1_p,v2_p)) * 180/np.pi
    print(f"Angle between geo-coord vectors: {a1}")
    print(f"Angle between  pointing vectors: {a2}")

    # v2_p = (R_y(a) @ v1_p.transpose()).transpose()
    print(v1_p)
    print(v2_p)

    gs_R_E_ = best_fit_rotation_two_vectors(v1_geo,v2_geo,v1_p,v2_p)

    # print(R_z(30*np.pi/180))
    print(np.round(gs_R_E_,4))

    print(v1_p)
    print((gs_R_E_ @ v1_geo.transpose()).transpose())

    print(v2_p)
    print((gs_R_E_ @ v2_geo.transpose()).transpose())

if __name__ == "__main__":
    main()
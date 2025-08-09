import sys
import numpy as np
import json

# check machanical pointing
# check rf pointing
# correct for tilt (WIP)
# get pointing test done, figure out the formula for the errors etc.??
# -> design method for pointing (laser pointer)

# lesson learned: inventory

def geo_to_ecef(lat, lon, alt):
    rad_lat = lat * (np.pi / 180.0)
    rad_lon = lon * (np.pi / 180.0)

    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    v = a / np.sqrt(1 - e2 * np.sin(rad_lat) * np.sin(rad_lat))

    return np.array([
        (v + alt) * np.cos(rad_lat) * np.cos(rad_lon),
        (v + alt) * np.cos(rad_lat) * np.sin(rad_lon),
        (v * (1 - e2) + alt) * np.sin(rad_lat)
    ])

def vec2XY(v): # pointing vector to XY
    v = v/np.linalg.norm(v)
    return (
        float(np.arcsin(-v[1]) * 180/np.pi),
        float(np.arcsin(v[0] / np.sqrt(1-v[1]*v[1] + 1e-20)) * 180/np.pi)
    )

# Rotation matricies
R_x = lambda a: np.array([
    [1,         0,         0],
    [0, np.cos(a),-np.sin(a)],
    [0, np.sin(a), np.cos(a)]
])
R_y = lambda a: np.array([
    [ np.cos(a), 0, np.sin(a)],
    [         0, 1,         0],
    [-np.sin(a), 0, np.cos(a)]
])
R_z = lambda a: np.array([
    [np.cos(a),-np.sin(a), 0],
    [np.sin(a), np.cos(a), 0],
    [0,         0,         1]
])

# scew-matrix operator
# scew = lambda v: np.array([
#     [    0,-v[2], v[1]],
#     [ v[2],    0,-v[0]],
#     [-v[1], v[0],    0]
# ])

# def aa2R(axis,angle): # angle axis to rotation matrix
#     v = axis/np.linalg.norm(axis)
#     return np.eye(3) + scew(v) * np.sin(angle) + (1-np.cos(angle)) * scew(v) @ scew(v)

xyz2yzx = np.array([
    [0,0,1],
    [1,0,0],
    [0,1,0]
])


def world2gs(lat : float, long : float): # rotation matrix from geographic coordinate system world frame to ground station frame
    return R_z(long*np.pi/180) @ R_y(-lat*np.pi/180) @ xyz2yzx @ R_z(90*np.pi/180)
    
def gs_tilt_correction(az_offset : float, tilt_axis_angle : float, tilt : float):
    return R_z(tilt_axis_angle*np.pi/180) @ R_y(tilt*np.pi/180) @ R_z(-az_offset*np.pi/180)

def main(argv : list[str]):
    # load ground station config
    fname = "gs_config_ratatosk.json"
    with open(fname, "r") as f:
        data = json.load(f)

    lat_gs = data["latitude"]
    long_gs = data["longitude"]
    alt_gs = data["altitude"]
    az_offset = data["azimuth_offset"]
    tilt_axis = data["tilt"]
    tilt = data["tilt_axis"]

    # Get the balloon coordinates from the input arguments
    lat_b,long_b,alt_b = (float(argv[1]),float(argv[2]),float(argv[3]))

    # Calculate the vectors (Earth Centered Earth Fixed frame)
    v_gs = geo_to_ecef(lat_gs,long_gs,alt_gs)
    v_b = geo_to_ecef(lat_b,long_b,alt_b)

    print(tuple(map(float,v_gs/1000000)))
    print(tuple(map(float,v_b/1000000)))

    # get the pointing vector between the ground station and the balloon
    v_point = v_b - v_gs
    dist = np.linalg.norm(v_point)
    v_point = v_point/dist

    # get the pointing vector in the ground station frame
    gs_v_point = np.transpose(world2gs(lat_gs, long_gs) @ gs_tilt_correction(az_offset, tilt_axis, tilt)) @ v_point

    # calculate the azimuth and elevation from the vector in the ground station frame
    X,Y = vec2XY(gs_v_point)
    print(X,Y, dist)


if __name__ == "__main__":
    main(sys.argv)
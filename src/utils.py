import numpy as np
import re,os
from pygeodesy import ellipsoidalVincenty as ev
from pygeodesy import GeoidEGM96


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

def XY2vec(X,Y,off=0):
    X = (X - 90) * np.pi/180; Y = (Y - 90) * np.pi/180; off *= np.pi/180
    return (R_y(Y) @ R_x(X) @ R_x(off) @ np.array([[0],[0],[1]])).transpose()[0]

def vec2XY(v, offset): # pointing vector to XY
    v = v/np.linalg.norm(v)
    if v[2] > 0:
        X = float((np.arcsin(-v[1]) - offset * np.pi/180) * 180/np.pi)
        Y = float(np.arcsin(v[0] / np.sqrt(1-v[1]*v[1] + 1e-20)) * 180/np.pi)
    else:
        X = float((np.pi - np.arcsin(-v[1]) - offset * np.pi/180) * 180/np.pi)
        Y = -float((np.arcsin(v[0] / np.sqrt(1-v[1]*v[1] + 1e-20))) * 180/np.pi)
    return (
        X + 90,
        Y + 90
    )

def geo_to_ecef(lat, lon, alt):
    alt = msl_to_ellipsoid(lat,lon,alt)

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

def best_fit_rotation_two_vectors(e1, e2, g1, g2):
    """
    Compute best-fit rotation R (3x3) such that R*[e1,e2] ≈ [g1,g2].
    Uses the Kabsch algorithm with two vector correspondences.
    """
    # stack into 3xN matrices
    E = np.column_stack((e1, e2))
    G = np.column_stack((g1, g2))

    # covariance
    C = G @ E.T   # 3x3

    # SVD
    U, _, Vt = np.linalg.svd(C)

    # ensure proper rotation (determinant +1)
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return R

def best_fit_rotation_n_vectors(es, gs, weights=None):
    """
    Compute best-fit rotation R (3x3) such that R*e_n ≈ g_n.
    Uses the Kabsch algorithm with two vector correspondences.
    """
    # stack into 3xN matrices
    E = np.column_stack(es)
    G = np.column_stack(gs)

    if weights is None:
        weights = np.ones(E.shape[1])
    weights = np.asarray(weights, dtype=float)
    weights /= np.sum(weights)

    # diagonal weighting matrix
    W = np.diag(weights)

    # weighted covariance
    C = G @ W @ E.T

    # SVD
    U, _, Vt = np.linalg.svd(C)

    # ensure proper rotation (determinant +1)
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return R

def lat_long_str_to_decimal(geo_str):
    card_dict = {"N":1,"S":-1,"E":1,"W":-1}
    s1, s2 = geo_str.strip().split(" ")
    deg,sec,min,card = re.findall(r"(?:[0-9]{1,3}(?:\.[0-9]+)*)|[NSEWnsew]",s1)
    lat = card_dict[card.upper()] * (int(deg) + int(sec)/60.0 + float(min)/3600.0)
    deg,sec,min,card = re.findall(r"(?:[0-9]{1,3}(?:\.[0-9]+)*)|[NSEWnsew]",s2)
    long = card_dict[card.upper()] * (int(deg) + int(sec)/60.0 + float(min)/3600.0)

    return lat,long
    
ginterpolator = GeoidEGM96(os.path.join(
        os.path.realpath(__file__).rstrip("/src/utils.py"),
        "resources",
        "WW15MGH.GRD"
    ))
def msl_to_ellipsoid(lat, lon, height_msl):
    point = ev.LatLon(lat, lon)
    assert point.height == 0
    height_ellipsoid = height_msl + ginterpolator(point)
    return height_ellipsoid


if __name__ == "__main__":
    print(msl_to_ellipsoid(67.88832704180142, 21.083030442478613,0))
    print(msl_to_ellipsoid(67.15339150829834, 28.090406042438797,0))
    print(lat_long_str_to_decimal("""67°52'45"N 21°03'41"E"""))

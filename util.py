import numpy as np

# calculates the nearest point between two skewed lines in 3D space
def nearest_point_between_skews(p1, p2, vec1, vec2):
    normal = np.cross(np.cross(vec1, vec2), vec2)
    delta = p2 - p1
    a = p1 + vec1 * (np.dot(delta, normal) / np.dot(vec1, normal))
    b = p2 + vec2 * (np.dot(a - p2, vec2) / np.dot(vec2, vec2))
    return (a + b) / 2

# returns least squares fit transform and rotation matrix between two sets points in 3D space
def calc_transform(a, b):
    aMean = np.mean(a, axis=0)
    bMean = np.mean(b, axis=0)

    aCentered = a - aMean
    bCentered = b - bMean

    c = np.dot(aCentered.T, bCentered) / len(a)
    v, s, w = np.linalg.svd(c)
    if np.linalg.det(v) * np.linalg.det(w) < 0:
        s[-1] *= -1
        v[:, -1] *= -1

    calcR = np.dot(v, w)
    return calcR, bMean - aMean @ calcR


Used with OpenCV 2:
 * Orb.cpp is a modified version of OpenCV2 Orb with FAST object from rtabmap (FAST with Grid adaptor).
 
 Used for all OpenCV versions:
 * solvepnp.cpp is a copy of the OpenCV3 version of solvePnPRansac.
 * five-point.cpp is a copy of the same file in OpenCV (d2872afce0fcc84a52b5753960730595550e1b62) but with RANSAC estimator using 6 points instead of 5 points to avoid "DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences. (expected: 'count >= 6')" error on recent OpenCV versions using DLT by default.
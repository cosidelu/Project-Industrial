"""
Camera utility functions for single frame capture and defect detection.
Wraps the functions from colored_defect_search.py for easy single-frame acquisition.
"""

import pyzed.sl as sl
from camera_libraries.colored_defect_search import (
    init_zed,
    zed_mat_to_bgr,
    find_green_mask_and_centroid,
    extract_3d_points_from_mask,
    compute_mean_3d_point
)

# ============================================================
# PARAMETER TUNING
# Adjust these values to tune the defect detection
# ============================================================

import camera_libraries.colored_defect_search as cds
import numpy as np

# HSV range for green detection
cds.LOWER_GREEN = np.array([35, 80, 80])
cds.UPPER_GREEN = np.array([85, 255, 255])

# Minimum blob area
cds.MIN_GREEN_AREA = 300

# Circular mask parameters
cds.CIRCULAR_MASK_RADIUS_RATIO = 0.2
cds.CIRCULAR_MASK_ENABLED = True

# 3D point extraction
cds.MAX_3D_SAMPLES = 200

# ============================================================
# GLOBAL CAMERA OBJECTS
# ============================================================

_zed_camera = None
_runtime = None
_image_zed = None
_point_cloud = None


def initialize_camera():
    """
    Initialize the ZED camera once at startup.
    Call this before capture_single_frame_and_detect().
    """
    global _zed_camera, _runtime, _image_zed, _point_cloud
    _zed_camera, _runtime, _image_zed, _point_cloud = init_zed()
    print("Camera initialized successfully")


def capture_single_frame_and_detect():
    """
    Capture a single frame and detect the green defect.
    
    Returns:
    - centroid_2d: (cx, cy) tuple or None if not found
    - mean_point_3d: [X, Y, Z] numpy array or None if not found
    """
    if _zed_camera is None:
        raise RuntimeError("Camera not initialized. Call initialize_camera() first.")
    
    if _zed_camera.grab(_runtime) != sl.ERROR_CODE.SUCCESS:
        return None, None
    
    # Acquire data
    _zed_camera.retrieve_image(_image_zed, sl.VIEW.LEFT)
    _zed_camera.retrieve_measure(_point_cloud, sl.MEASURE.XYZRGBA)
    
    # Convert to BGR
    bgr_image = zed_mat_to_bgr(_image_zed)
    
    # Detect green defect
    mask, centroid_2d, contour = find_green_mask_and_centroid(bgr_image)
    
    # Extract and compute 3D points
    mean_point_3d = None
    if centroid_2d is not None:
        points_3d = extract_3d_points_from_mask(mask, _point_cloud)
        if len(points_3d) > 0:
            mean_point_3d = compute_mean_3d_point(points_3d)
    
    return centroid_2d, mean_point_3d


def close_camera():
    """
    Close the camera when done.
    """
    global _zed_camera
    if _zed_camera is not None:
        _zed_camera.close()
        _zed_camera = None
        print("Camera closed")


# ============================================================
# EXAMPLE USAGE
# ============================================================

if __name__ == "__main__":
    # Initialize camera
    initialize_camera()
    
    # Capture and detect
    centroid_2d, mean_point_3d = capture_single_frame_and_detect()
    
    print(f"Centroide 2D: {centroid_2d}")      # (cx, cy)
    print(f"Posizione 3D: {mean_point_3d}")    # [X, Y, Z]
    
    # Close camera
    close_camera()

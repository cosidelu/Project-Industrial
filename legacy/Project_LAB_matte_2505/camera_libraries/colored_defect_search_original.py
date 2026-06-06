import cv2
import numpy as np
import pyzed.sl as sl

# ============================================================
# PARAMETERS TO TUNE
# ============================================================

# Initial HSV range for green detection
LOWER_GREEN = np.array([35, 80, 80])
UPPER_GREEN = np.array([85, 255, 255])

# Minimum blob area to accept a green region as valid
MIN_GREEN_AREA = 300

# ============================================================
# CAMERA / ZED UTILITY FUNCTIONS
# They provide the correct syntax to initialize and read data
# from the ZED camera.
# ============================================================

def init_zed():
    """
    This function initializes the ZED camera with standard settings
    for this lab and returns all the main objects needed later.

    Returned objects:
    - zed: the camera object
    - runtime: runtime parameters used during grabbing
    - image_zed: container for the RGB image
    - point_cloud: container for the 3D point cloud
    """
    zed = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED opening error: {status}")

    runtime = sl.RuntimeParameters()

    image_zed = sl.Mat()
    point_cloud = sl.Mat()

    return zed, runtime, image_zed, point_cloud


def zed_mat_to_bgr(image_zed):
    """

    This function converts the ZED image format into a standard OpenCV
    BGR image so that it can be processed with cv2 functions.

    Input:
    - image_zed: ZED image container

    Output:
    - image in OpenCV BGR format
    """
    image = image_zed.get_data()
    if image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


# ============================================================
#                 2D GREEN DEFECT DETECTION
# ============================================================

def find_green_mask_and_centroid(bgr_image):
    """

    Goal:
    Detect the green defect in the RGB image.

    Suggested steps:
    1. Convert the image from BGR to HSV
    2. Threshold the image using the green HSV range
    3. Clean the mask if needed
    4. Find connected regions / contours
    5. Keep the main green blob
    6. Compute its 2D centroid

    Input:
    - bgr_image: standard OpenCV image

    Output:
    - mask: binary mask of detected green pixels
    - centroid: (cx, cy) or None if not found
    - contour: main contour or None

    TO DO:
    - improve the robustness of this function and look for green pixels only in 
    in the area where you expect the helmet. There might be other green objects in 
    the background.
    """
 
    # Convert BGR image to HSV
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to keep only green pixels and convert it into a binary mask
    # A mask is a black and white image where white pixels are those that match the condition
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    # Optional mask cleaning

    # Select the dimension of the brush (Increase to get a smoother mask, decrease to catch smaller features)
    kernel = np.ones((5, 5), np.uint8)
    # Remove isolated white pixels (noise in the backgroound)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Remove isolated black pixels (holes in the "defect" or irregular contour of the "defect")
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the binary mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If no contour is found, return failure
    if not contours:
        return mask, None, None

    # Select the contour with the largest area as the candidate defect
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    # Reject very small regions
    if area < MIN_GREEN_AREA:
        return mask, None, None

    # Compute the centroid of the selected contour
    M = cv2.moments(largest)

    # m00 = area of the contour
    # m10 = first order moment about y-axis
    # m01 = first order moment about x-axis

    if M["m00"] == 0:
        return mask, None, None

    # Compute the geometric center of the contour
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return mask, (cx, cy), largest


# ============================================================
# PART 2 - 3D POSITION ESTIMATION
# ============================================================

def extract_3d_points_from_mask(mask, point_cloud):
    """

    Goal:
    Convert all green pixels into valid 3D points.

    Important note:
    Do NOT use only one pixel if you want a stable 3D estimate, but collect many 
    3D points and average them.

    1. Get all pixel coordinates from the binary mask
    2. For each pixel, read the corresponding 3D point from the ZED point cloud
    3. Reject invalid 3D points
    4. Return the list of valid 3D points

    Input:
    - mask: binary mask of green pixels
    - point_cloud: ZED point cloud container

    Output:
    - Nx3 numpy array of valid 3D points
    """

    # Extract all pixel coordinates where the mask is white
    ys, xs = np.where(mask > 0)

    points_3d = []

    # For each green pixel, query the corresponding 3D point
    for x, y in zip(xs, ys):
        err, point = point_cloud.get_value(int(x), int(y))

        if err == sl.ERROR_CODE.SUCCESS:
            X, Y, Z, RGBA = point

            # Keep only finite and valid 3D points
            if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                if Z > 0:
                    points_3d.append([X, Y, Z])

    if len(points_3d) == 0:
        return np.empty((0, 3), dtype=np.float32)

    return np.array(points_3d, dtype=np.float32)


def compute_mean_3d_point(points_3d):
    """
    Goal:
    Compute one stable 3D estimate from many valid 3D points.

    Input:
    - points_3d: Nx3 array

    Output:
    - mean_point: [X, Y, Z] or None if no valid points exist
    """

    if len(points_3d) == 0:
        return None

    mean_point = np.mean(points_3d, axis=0)
    return mean_point


# ============================================================
# DEBUG / VISUALIZATION FUNCTIONS
# ============================================================

def draw_debug(bgr_image, mask, centroid, contour, mean_point_3d):
    """
    This function overlays visual debug information on the image:
    - detected contour
    - centroid
    - estimated 3D point

    It also converts the binary mask to BGR so that it can be displayed
    in a separate OpenCV window.

    Input:
    - bgr_image: original RGB image in OpenCV format
    - mask: binary mask
    - centroid: 2D centroid or None
    - contour: main contour or None
    - mean_point_3d: estimated mean 3D point or None

    Output:
    - debug image with overlays
    - mask image in BGR format
    """
    debug = bgr_image.copy()

    if contour is not None:
        cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)

    if centroid is not None:
        cx, cy = centroid
        cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
        cv2.putText(
            debug,
            f"center: ({cx}, {cy})",
            (cx + 10, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    if mean_point_3d is not None:
        X, Y, Z = mean_point_3d
        cv2.putText(
            debug,
            f"3D = ({X:.3f}, {Y:.3f}, {Z:.3f}) m",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    return debug, mask_bgr


# ============================================================
# MAIN PROGRAM
# 1. acquire image
# 2. acquire point cloud
# 3. detect green area in 2D
# 4. estimate 3D position
# ============================================================

def main():
    zed, runtime, image_zed, point_cloud = init_zed()

    print("Press ESC to quit.")

    try:
        while True:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                # ------------------------------------------------
                # ZED DATA ACQUISITION
                # ------------------------------------------------

                # Acquire an image from the cameras
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)  
                # Estimate depth and generate a pointcloud            
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                # Convert the ZED pointcloud format to the Open-CV format 
                bgr_image = zed_mat_to_bgr(image_zed)

                # ------------------------------------------------
                # 2D GREEN DEFECT DETECTION
                # ------------------------------------------------
                mask, centroid, contour = find_green_mask_and_centroid(bgr_image)

                # ------------------------------------------------
                # 3D POSITION ESTIMATION
                # ------------------------------------------------
                mean_point_3d = None

                if centroid is not None:
                    # Suggested strategy:
                    # use all green pixels from the mask, not only the centroid
                    points_3d = extract_3d_points_from_mask(mask, point_cloud)

                    if len(points_3d) > 0:
                        mean_point_3d = compute_mean_3d_point(points_3d)

                # ------------------------------------------------
                # DEBUG VISUALIZATION
                # ------------------------------------------------
                debug_img, mask_bgr = draw_debug(
                    bgr_image, mask, centroid, contour, mean_point_3d
                )

                cv2.imshow("RGB Debug", debug_img)
                cv2.imshow("Green Mask", mask_bgr)

                # ------------------------------------------------
                # TERMINAL OUTPUT
                # ------------------------------------------------
                if mean_point_3d is not None:
                    X, Y, Z = mean_point_3d
                    print(
                        f"Estimated 3D point: X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m",
                        end="\r"
                    )
                else:
                    print(
                        "Green defect not found or invalid 3D points.",
                        end="\r"
                    )

                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break

    finally:
        zed.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
import cv2
import numpy as np
import pyzed.sl as sl
import os
from datetime import datetime


def mat_to_bgr(zed_mat):
    img = zed_mat.get_data()
    if img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def depth_to_display(depth_np):
    depth_clean = np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)

    valid = depth_clean > 0
    if np.any(valid):
        dmin = np.min(depth_clean[valid])
        dmax = np.max(depth_clean[valid])
        if dmax > dmin:
            depth_norm = np.zeros_like(depth_clean, dtype=np.uint8)
            depth_norm[valid] = ((depth_clean[valid] - dmin) / (dmax - dmin) * 255).astype(np.uint8)
        else:
            depth_norm = np.zeros_like(depth_clean, dtype=np.uint8)
    else:
        depth_norm = np.zeros_like(depth_clean, dtype=np.uint8)

    return cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)


def create_output_folder():
    base_folder = "captures"
    if not os.path.exists(base_folder):
        os.mkdir(base_folder)
    return base_folder


def get_timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def main():
    zed = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER

    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Error opening ZED")
        return

    runtime = sl.RuntimeParameters()

    image_zed = sl.Mat()
    depth_zed = sl.Mat()
    point_cloud = sl.Mat()

    output_folder = create_output_folder()

    print("Press SPACE to save, ESC to exit")

    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            rgb = mat_to_bgr(image_zed)
            depth_np = depth_zed.get_data()
            depth_vis = depth_to_display(depth_np)

            rgb_small = cv2.resize(rgb, (640, 360))
            depth_small = cv2.resize(depth_vis, (640, 360))
            rgbd = np.hstack((rgb_small, depth_small))

            cv2.imshow("RGB", rgb_small)
            cv2.imshow("Depth", depth_small)
            cv2.imshow("RGBD", rgbd)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                break

            if key == 32:  # SPACE
                ts = get_timestamp()

                rgb_path = os.path.join(output_folder, f"rgb_{ts}.png")
                depth_path = os.path.join(output_folder, f"depth_{ts}.png")
                rgbd_path = os.path.join(output_folder, f"rgbd_{ts}.png")
                npz_path = os.path.join(output_folder, f"rgbd_{ts}.npz")
                pc_path = os.path.join(output_folder, f"pointcloud_{ts}.ply")

                cv2.imwrite(rgb_path, rgb)
                cv2.imwrite(depth_path, depth_vis)
                cv2.imwrite(rgbd_path, rgbd)

                np.savez(npz_path, rgb=rgb, depth=depth_np)

                err = point_cloud.write(pc_path)

                print("\nSaved:")
                print(rgb_path)
                print(depth_path)
                print(rgbd_path)
                print(npz_path)

                if err == sl.ERROR_CODE.SUCCESS:
                    print(pc_path)
                else:
                    print("Error saving point cloud:", err)

                # distanza punto centrale
                cx = rgb.shape[1] // 2
                cy = rgb.shape[0] // 2
                err, point = point_cloud.get_value(cx, cy)

                if err == sl.ERROR_CODE.SUCCESS:
                    x, y, z, _ = point
                    print(f"Distance from center: {np.sqrt(x*x + y*y + z*z):.3f} m")

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
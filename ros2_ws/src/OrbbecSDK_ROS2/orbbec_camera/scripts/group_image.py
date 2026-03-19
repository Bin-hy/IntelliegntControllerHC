import os
import shutil
import re
from collections import defaultdict
from pathlib import Path

# Dynamically determine the project root directory
# Script location: .../ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/group_image.py
# We want to go up to the project root (parent of ros2_ws)
# parents[0] = scripts
# parents[1] = orbbec_camera
# parents[2] = OrbbecSDK_ROS2
# parents[3] = src
# parents[4] = ros2_ws
# parents[5] = Project Root (e.g., IntelliegntControllerHC)

try:
    current_file_path = Path(__file__).resolve()
    # Go up 6 levels to reach the project root directory
    project_root = current_file_path.parents[5]
    image_directory = str(project_root / "orbbec_images")
except Exception as e:
    # Fallback to current directory if path resolution fails
    print(f"Warning: Could not resolve project root: {e}. Using current directory.")
    image_directory = os.path.abspath("orbbec_images")

print(f"Orbbec Image Directory: {image_directory}")

time_diff_threshold = 100
sync_time_diff_threshold = 33
current_path = os.path.dirname(os.path.abspath(__file__))
master_camera_serial_no = None
use_device_time = True
time_domain = "system_timestamp" if not use_device_time else "hardware_timestamp"


def image_hash(image_info):
    assert image_info is not None
    return (
        f"{image_info['serial_no']}_{image_info['index']}_{image_info['stream_name']}"
    )


def parse_image_filename(filename):
    parts = filename.split("_")
    return {
        "stream_name": parts[0],
        "index": int(parts[1]),
        "system_timestamp": float(parts[2]),
        "hardware_timestamp": float(parts[3]),
        "resolution": parts[4],
        "fps": int(parts[5].split("hz")[0]),
    }


def analyze_images():
    if not os.path.exists(image_directory):
        print(f"Image directory does not exist: {image_directory}")
        return {}
        
    serial_dirs = [
        os.path.join(image_directory, d) for d in os.listdir(image_directory)
        if os.path.isdir(os.path.join(image_directory, d))
    ]
    images = defaultdict(list)

    for serial_dir in serial_dirs:
        for filename in os.listdir(serial_dir):
            if filename.endswith(".png"):
                image_info = parse_image_filename(filename)
                image_info["serial_no"] = os.path.basename(serial_dir)
                image_info["path"] = os.path.join(serial_dir, filename)
                images[image_info["serial_no"]].append(image_info)

    return images


def copy_images_to_grouped_directory(image_group, index, grouped_image_directory):
    if len(image_group) <= 1:
        return
    ref_image = image_group[0]

    for image in image_group:
        time_diff = int(image[time_domain] - ref_image[time_domain])
        origin_filename = re.split(r"\.", os.path.basename(image["path"]))[0]
        suffix = (
            "_ref"
            if image["serial_no"] == ref_image["serial_no"]
            and image["stream_name"] == "color"
            else ""
        )
        status = "_anomaly" if abs(time_diff) > sync_time_diff_threshold else ""

        grouped_image_path = os.path.join(
            grouped_image_directory,
            f"{index}_{origin_filename}_{image['serial_no']}_[{time_diff}]{suffix}{status}.png",
        )

        shutil.copy(image["path"], grouped_image_path)


def group_images_by_time(all_images):
    if not all_images:
        return

    grouped_image_directory = os.path.join(current_path, "grouped_images")
    os.makedirs(grouped_image_directory, exist_ok=True)
    reference_serial_no = master_camera_serial_no or list(all_images.keys())[0]
    
    if reference_serial_no not in all_images:
        return

    reference_images = [
        img for img in all_images[reference_serial_no] if img["stream_name"] == "color"
    ]
    reference_images.sort(key=lambda x: int(x[time_domain]))

    for index, ref_image in enumerate(reference_images):
        image_group = [ref_image]

        for serial_no, images_list in all_images.items():
            min_diffs = {name: float("inf") for name in ["color", "depth"]}
            min_images = {name: None for name in ["color", "depth"]}

            for image in images_list:
                if serial_no == reference_serial_no and image["stream_name"] == "color":
                    continue

                time_diff = float(abs(image[time_domain] - ref_image[time_domain]))
                stream_name = image["stream_name"]

                if time_diff < min_diffs[stream_name]:
                    min_diffs[stream_name] = time_diff
                    min_images[stream_name] = image
            
            if min_images["color"]:
                image_group.append(min_images["color"])
            if min_images["depth"]:
                image_group.append(min_images["depth"])

        copy_images_to_grouped_directory(image_group, index, grouped_image_directory)

if __name__ == "__main__":
    images = analyze_images()
    group_images_by_time(images)

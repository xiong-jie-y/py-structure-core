import json
import os
import time
from enum import IntEnum
from os.path import join

import click
import cv2
import open3d as o3d
import numpy as np

import structure_core

DEFAULT_CONFIG_CONTENT = {
    "name": "Realsense bag file",
    "path_dataset": "dataset/structurecore",
    "path_intrinsic": "dataset/structurecore/camera_intrinsic.json",
    "max_depth": 5.0,
    "voxel_size": 0.05,
    "max_depth_diff": 0.07,
    "preference_loop_closure_odometry": 0.1,
    "preference_loop_closure_registration": 5.0,
    "tsdf_cubic_size": 3.0,
    "icp_method": "color",
    "global_registration": "ransac",
    "python_multi_threading": True,
    "n_frames_per_fragment": 100,
    "make_point_cloud": True
}

class StructureCoreDepthRangeMode(IntEnum):
    VeryShort = 0
    Short = 1
    Medium = 2
    Long = 3
    VeryLong = 4
    Hybrid = 5
    BodyScanning = 5
    Default = 5

class StructureCoreDepthResolution(IntEnum):
    QVGA = 0
    VGA = 1
    SXGA = 2


class StructureCoreReaderWrapper:
    def __init__(self):
        self.reader = structure_core.StructureCoreReader(
            StructureCoreDepthRangeMode.VeryShort, 
            StructureCoreDepthResolution.SXGA
        )

    def get_realsense_compatible_frame(self):
        arr, depth = self.reader.get_latest_frame()
        arr2 = arr.reshape((480, 640, 3))
        arr2_bgr = cv2.cvtColor(arr2, cv2.COLOR_RGB2BGR)

        depth = depth.reshape((480, 640))

        depth *= 1000
        depth = depth.astype(np.uint16)

        return arr2, arr2_bgr, depth

    def get_intrinsic(self):
        intr = self.reader.get_intrinsic()
        return o3d.camera.PinholeCameraIntrinsic(intr['width'], intr['height'], intr['intrinsic_matrix'][0],
                                            intr['intrinsic_matrix'][4], intr['intrinsic_matrix'][6],
                                            intr['intrinsic_matrix'][7])

class Open3DRGBDDataset:
    def __init__(self, output_dir):
        color_path = join(output_dir, "color")
        os.makedirs(color_path, exist_ok=True)
        depth_path = join(output_dir, "depth")
        os.makedirs(depth_path, exist_ok=True)
        json.dump(DEFAULT_CONFIG_CONTENT, open(join(output_dir, "structurecore.json"), "w"))

        self.color_path = color_path
        self.depth_path = depth_path
        self.output_dir = output_dir

    def output_intrinsic(self, intr):
        json.dump(intr, open(join(self.output_dir, "camera_intrinsic.json"), "w"))

    def output_frame(self, frame_count, arr2_bgr, depth):
        cv2.imwrite("%s/%06d.png" % \
                (self.depth_path, frame_count), depth)
        cv2.imwrite("%s/%06d.jpg" % \
                (self.color_path, frame_count), arr2_bgr)

def create_dataset(dataset_type, output_dir):
    if dataset_type == "open3d":
        return Open3DRGBDDataset(output_dir)

@click.command()
@click.option('--output-dir')
@click.option('--output-format', default="open3d")
def run(output_dir, output_format):
    reader = StructureCoreReaderWrapper()

    time.sleep(3)

    dataset = create_dataset(output_format, output_dir)
    dataset.output_intrinsic(reader.reader.get_intrinsic())

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    frame_count = 0
    pcd = o3d.geometry.PointCloud()
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    while True:
        if reader.reader.new_frame_arrived():
            arr2, arr2_bgr, depth = reader.get_realsense_compatible_frame()

            color_image = o3d.geometry.Image(arr2)
            depth_image = o3d.geometry.Image(depth)

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1000,
                depth_trunc=5,
                convert_rgb_to_intensity=False)

            out = reader.get_intrinsic()

            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, out)
            temp.transform(flip_transform)
            pcd.points = temp.points
            pcd.colors = temp.colors

            dataset.output_frame(frame_count, arr2_bgr, depth)

            if frame_count == 0:
                vis.add_geometry(pcd)

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            cv2.imshow("Color", arr2)
            cv2.imshow("Depth", depth)
            key = cv2.waitKey(1)

            # When ESC pressed.
            if key == 27:
                break

            frame_count += 1

if __name__ == "__main__":
    run()

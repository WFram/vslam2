import open3d as o3d
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path_to_pcd")
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.path_to_pcd)
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    o3d.visualization.draw_geometries([downpcd])

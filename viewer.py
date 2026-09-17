"""View point clouds saved by main.py.

Usage:
    python viewer.py
    python viewer.py /path/to/point-cloud.pcd
"""

import argparse
from pathlib import Path
import platform

import open3d as o3d
import yaml


CONFIG_PATH = Path(__file__).parent / "config" / "lidar.yaml"


def get_root_path() -> Path:
    """Return the dataset root used by main.py."""
    if platform.system() == "Windows":
        return Path("D:/")
    if platform.system() == "Linux":
        return Path("/mnt/d/")
    raise RuntimeError(f"Unsupported platform: {platform.system()}")


def get_default_pcd_path() -> Path:
    """Return the newest PCD file from the configured output directory."""
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        output_config = yaml.safe_load(config_file)["output"]

    output_dir = get_root_path() / output_config["directory"]
    pcd_paths = sorted(output_dir.glob(f"*{output_config['extension']}"))
    if not pcd_paths:
        raise FileNotFoundError(f"No PCD files found in: {output_dir}")

    return max(pcd_paths, key=lambda path: path.stat().st_mtime)


def main() -> None:
    parser = argparse.ArgumentParser(description="View a saved PCD point cloud.")
    parser.add_argument(
        "pcd_path",
        nargs="?",
        type=Path,
        help="PCD file to open (defaults to the newest configured output file).",
    )
    args = parser.parse_args()

    pcd_path = args.pcd_path or get_default_pcd_path()
    if not pcd_path.is_file():
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")

    point_cloud = o3d.io.read_point_cloud(str(pcd_path))
    if point_cloud.is_empty():
        raise ValueError(f"PCD file has no points: {pcd_path}")

    print(f"PCD: {pcd_path}")
    print(f"Points: {len(point_cloud.points)}")
    o3d.visualization.draw_geometries(
        [point_cloud],
        window_name=f"PCD Viewer - {pcd_path.name}",
    )


if __name__ == "__main__":
    main()

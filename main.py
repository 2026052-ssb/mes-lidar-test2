from pathlib import Path

import open3d as o3d
import platform

ROOT_PATH = Path("")
if platform.system() == "Windows":
    ROOT_PATH = Path("D:/")
elif platform.system() == "Linux":
    ROOT_PATH = Path("/mnt/d/")
else:
    raise Exception(f"Unsupported platform: {platform.system()}")

TARGET_PATH = ROOT_PATH / "datasets/mes-lidar-test1/cad"

# =========================
# CAD FBX
# =========================
target_path = str(TARGET_PATH / "2540_281_000.fbx")
print("FBX:", target_path)
mesh = o3d.io.read_triangle_mesh(target_path)

mesh.compute_vertex_normals()

o3d.visualization.draw([mesh])

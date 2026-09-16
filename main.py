from pathlib import Path
import platform
import numpy as np
import open3d as o3d


# =========================
# Path
# =========================
ROOT_PATH = Path("")
if platform.system() == "Windows":
    ROOT_PATH = Path("D:/")
elif platform.system() == "Linux":
    ROOT_PATH = Path("/mnt/d/")
else:
    raise Exception(f"Unsupported platform: {platform.system()}")

TARGET_PATH = ROOT_PATH / "datasets/mes-lidar-test1/cad"

target_path = str(TARGET_PATH / "2540_281_000.fbx")

print("FBX:", target_path)


# =========================
# 1. CAD → Triangle Mesh
# =========================
mesh = o3d.io.read_triangle_mesh(target_path)

mesh.compute_vertex_normals()

print(mesh)
print("Vertices :", np.asarray(mesh.vertices).shape)
print("Triangles:", np.asarray(mesh.triangles).shape)


# =========================
# 2. LiDAR 설정
# =========================

# LiDAR 위치
LIDAR_POSITION = np.array([
    0.0,   # X
    0.0,   # Y
    5.0    # Z
], dtype=np.float32)

# LiDAR 수평 FOV
H_FOV = 360.0

# LiDAR 수직 FOV
V_FOV_UP = 15.0
V_FOV_DOWN = -15.0

# LiDAR channel 수
CHANNELS = 16

# 수평 resolution
H_RESOLUTION = 0.2

# 최대 측정 거리
MAX_RANGE = 100.0


# =========================
# 3. Open3D Raycasting Scene
# =========================

scene = o3d.t.geometry.RaycastingScene()

# legacy TriangleMesh → tensor TriangleMesh
tmesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

geometry_id = scene.add_triangles(tmesh)

print("Raycasting scene ready.")


# =========================
# 4. LiDAR Ray 생성
# =========================

# 수평 방향
azimuths = np.arange(
    -H_FOV / 2,
    H_FOV / 2,
    H_RESOLUTION
)

# 수직 방향
elevations = np.linspace(
    V_FOV_DOWN,
    V_FOV_UP,
    CHANNELS
)

print("Horizontal rays:", len(azimuths))
print("Vertical channels:", len(elevations))
print("Total rays:", len(azimuths) * len(elevations))


# =========================
# 5. Ray direction 계산
# =========================

azimuth_rad = np.deg2rad(azimuths)
elevation_rad = np.deg2rad(elevations)

az, el = np.meshgrid(
    azimuth_rad,
    elevation_rad
)

# LiDAR 좌표계
#
# X → forward
# Y → left
# Z → up
#
# direction:
#
# x = cos(el) * cos(az)
# y = cos(el) * sin(az)
# z = sin(el)

directions = np.stack(
    [
        np.cos(el) * np.cos(az),
        np.cos(el) * np.sin(az),
        np.sin(el),
    ],
    axis=-1
)

directions = directions.reshape(-1, 3)


# =========================
# 6. Ray 생성
# =========================

origins = np.repeat(
    LIDAR_POSITION[None, :],
    len(directions),
    axis=0
)

# Open3D RaycastingScene 형식
# [ox, oy, oz, dx, dy, dz]

rays = np.hstack([
    origins,
    directions
]).astype(np.float32)

rays = o3d.core.Tensor(
    rays,
    dtype=o3d.core.Dtype.Float32
)


# =========================
# 7. Ray Casting
# =========================

ans = scene.cast_rays(rays)

# hit distance
t_hit = ans["t_hit"].numpy()


# =========================
# 8. 교차점 계산
# =========================

valid = np.isfinite(t_hit)

# 최대 거리 제한
valid &= t_hit <= MAX_RANGE

hit_distances = t_hit[valid]
hit_directions = directions[valid]

points = (
    origins[valid]
    + hit_directions * hit_distances[:, None]
)


print("Valid points:", len(points))


# =========================
# 9. Point Cloud 생성
# =========================

pcd = o3d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(points)

# =========================
# 10. LiDAR 위치 표시
# =========================

lidar_marker = o3d.geometry.TriangleMesh.create_sphere(
    radius=0.2
)

lidar_marker.translate(LIDAR_POSITION)

lidar_marker.paint_uniform_color(
    [1.0, 0.0, 0.0]
)


# =========================
# 11. Visualization
# =========================

o3d.visualization.draw(
    [
        mesh,
        pcd,
        lidar_marker,
    ]
)
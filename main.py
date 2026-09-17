from pathlib import Path
import platform
import numpy as np
import open3d as o3d
import yaml


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
CONFIG_PATH = Path(__file__).parent / "config" / "lidar.yaml"


def load_lidar_config(config_path: Path) -> dict:
    """Load LiDAR parameters from a YAML configuration file."""
    with config_path.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict):
        raise ValueError(f"Invalid LiDAR config: {config_path}")

    return config


def create_ray_lineset(
    ray_origins: np.ndarray,
    hit_points: np.ndarray,
    max_rays: int,
    color: list[float],
) -> o3d.geometry.LineSet:
    """Create a LineSet between LiDAR origins and ray hit points."""
    ray_count = min(len(hit_points), max_rays)
    sample_indices = np.linspace(
        0,
        len(hit_points) - 1,
        ray_count,
        dtype=int,
    )

    sampled_origins = ray_origins[sample_indices]
    sampled_points = hit_points[sample_indices]
    lineset = o3d.geometry.LineSet()
    lineset.points = o3d.utility.Vector3dVector(
        np.vstack([sampled_origins, sampled_points])
    )
    lineset.lines = o3d.utility.Vector2iVector(
        np.column_stack([
            np.arange(ray_count),
            np.arange(ray_count) + ray_count,
        ])
    )
    lineset.colors = o3d.utility.Vector3dVector(
        np.tile(color, (ray_count, 1))
    )

    return lineset


def save_point_cloud(point_cloud: o3d.geometry.PointCloud, output_path: Path) -> None:
    """Save a point cloud as a PCD file, creating its parent directory if needed."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not o3d.io.write_point_cloud(str(output_path), point_cloud, compressed=True):
        raise OSError(f"Failed to save point cloud: {output_path}")

    print(f"PCD saved: {output_path}")


lidar_config = load_lidar_config(CONFIG_PATH)

target_path = TARGET_PATH / "2540_281_000.fbx"
output_config = lidar_config["output"]
pcd_output_path = (
    ROOT_PATH
    / output_config["directory"]
    / f"{target_path.stem}{output_config['extension']}"
)

print("FBX:", target_path)


# =========================
# 1. CAD → Triangle Mesh
# =========================
mesh = o3d.io.read_triangle_mesh(str(target_path))

mesh.compute_vertex_normals()

print(mesh)
print("Vertices :", np.asarray(mesh.vertices).shape)
print("Triangles:", np.asarray(mesh.triangles).shape)


# =========================
# 2. LiDAR 설정
# =========================

vertices = np.asarray(mesh.vertices)

bbox_min = vertices.min(axis=0)
bbox_max = vertices.max(axis=0)
bbox_center = (bbox_min + bbox_max) / 2

print("Min:", bbox_min)
print("Max:", bbox_max)
print("Center:", bbox_center)


# LiDAR 위치
LIDAR_POSITION = np.array([
    bbox_max[0] + lidar_config["position"]["x_offset_from_max"],
    bbox_center[1] + lidar_config["position"]["y_offset_from_center"],
    bbox_center[2] + lidar_config["position"]["z_offset_from_center"],
], dtype=np.float32)

print("LiDAR position:", LIDAR_POSITION)

# LiDAR 수평 FOV
H_FOV = lidar_config["horizontal_fov_deg"]

# LiDAR 수직 FOV
V_FOV_UP = lidar_config["vertical_fov_deg"]["up"]
V_FOV_DOWN = lidar_config["vertical_fov_deg"]["down"]

# LiDAR channel 수
CHANNELS = lidar_config["channels"]

# 수평 resolution
H_RESOLUTION = lidar_config["horizontal_resolution_deg"]

# 최대 측정 거리
MAX_RANGE = lidar_config["max_range"]


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
        -np.cos(el) * np.cos(az),
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
# 9. LiDAR Ray 시각화
# =========================

ray_visualization_config = lidar_config["ray_visualization"]
ray_lines = None
if ray_visualization_config["enabled"] and len(points) > 0:
    ray_lines = create_ray_lineset(
        origins[valid],
        points,
        ray_visualization_config["max_rays"],
        ray_visualization_config["color"],
    )
    print("Visualized rays:", len(ray_lines.lines))


# =========================
# 10. Point Cloud 생성
# =========================

pcd = o3d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(points)

if output_config["enabled"]:
    save_point_cloud(pcd, pcd_output_path)

# =========================
# 12. LiDAR 위치 표시
# =========================

lidar_marker = o3d.geometry.TriangleMesh.create_sphere(
    radius=lidar_config["marker_radius"]
)

lidar_marker.translate(LIDAR_POSITION)

lidar_marker.paint_uniform_color(
    [1.0, 0.0, 0.0]
)


# =========================
# 13. Visualization
# =========================

geometries = [
    # mesh,           # CAD
    pcd,            # PCD
    lidar_marker
]
if ray_lines is not None:
    geometries.append(ray_lines)

o3d.visualization.draw_geometries(geometries)

import numpy as np
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation as R

# 路径
data_dir = "data/2025-06-26_07.51.22"
pcd_path = f"{data_dir}/pointcloud.ply"
poses_path = f"{data_dir}/undistorted/image-poses.txt"
images_dir = f"{data_dir}/undistorted"

# 指定使用的图像名
selected_img_name = "00115-cam3.jpg"  # 请替换为实际的图像名

# 内参
fx, fy, cx, cy = 900, 900, 2028, 1520
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

# 读取点云
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)
# 降采样10倍
points = points[::5]
colors = np.zeros((len(points), 3))

# 读取poses
poses_data = np.genfromtxt(poses_path, dtype=str, delimiter=' ', skip_header=1)
poses = {}
for line in poses_data:
    cam_id, img_name, x, y, z, rw, rx, ry, rz = line
    cam_id = int(cam_id)
    x, y, z, rw, rx, ry, rz = map(float, [x, y, z, rw, rx, ry, rz])
    # print(f"读取相机 {cam_id} 图像 {img_name} 位姿")
    if cam_id not in poses:
        poses[cam_id] = []
    poses[cam_id].append((img_name, np.array([x, y, z]), np.array([rw, rx, ry, rz])))

# 查找指定的图像
selected_entry = None
for cam_id in poses:
    for entry in poses[cam_id]:
        if entry[0] == selected_img_name:
            selected_entry = entry
            break
    if selected_entry:
        break

if selected_entry is None:
    raise ValueError(f"指定的图像 {selected_img_name} 未找到")

selected_pos, selected_quat = selected_entry[1], selected_entry[2]

# 加载选定的图像
selected_img = cv2.imread(f"{images_dir}/{selected_img_name}")

print(f"使用图像: {selected_img_name} 进行着色")
print(f"相机位置: {selected_pos}, 四元数: {selected_quat}")

# 为每个点着色，使用选定的相机
for i, point in enumerate(points):
    dist = np.linalg.norm(point - selected_pos)
    if dist > 5.0:  # 超过5米不考虑
        continue

    # 计算投影
    rot = R.from_quat([selected_quat[1], selected_quat[2], selected_quat[3], selected_quat[0]]).as_matrix()  # rx, ry, rz, rw
    R_mat = rot
    t = selected_pos
    # 世界到相机坐标
    point_cam = R_mat.T @ (point - t)
    if point_cam[2] > 0:  # 在相机前面
        uv = K @ point_cam / point_cam[2]
        u, v = uv[0], uv[1]
        h, w = selected_img.shape[:2]
        if 0 <= u < w and 0 <= v < h:
            color = selected_img[int(v), int(u)] / 255.0
            colors[i] = [color[2], color[1], color[0]]  # BGR to RGB
    
    if i % 10000 == 0:
        print(f"进度: {i}/{len(points)}")

pcd.colors = o3d.utility.Vector3dVector(colors)
pcd.points = o3d.utility.Vector3dVector(points)  # 更新点云
o3d.io.write_point_cloud("colored_pointcloud.pcd", pcd)
print("着色完成，已保存为 colored_pointcloud.pcd")

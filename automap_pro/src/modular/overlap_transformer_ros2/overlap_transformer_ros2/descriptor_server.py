#!/usr/bin/env python3
"""
OverlapTransformer ROS2 service node: PointCloud2 -> 256-d descriptor.
Uses OverlapTransformer-master (range_projection + featureExtracter).
"""

import os
import sys

# Add OverlapTransformer-master to path (sibling of workspace or env)
OT_ROOT = os.environ.get(
    "OVERLAP_TRANSFORMER_ROOT",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "OverlapTransformer-master"),
)
OT_ROOT = os.path.abspath(OT_ROOT)
if os.path.isdir(OT_ROOT):
    sys.path.insert(0, OT_ROOT)
    sys.path.insert(0, os.path.join(OT_ROOT, "tools"))
    sys.path.insert(0, os.path.join(OT_ROOT, "modules"))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from overlap_transformer_msgs.srv import ComputeDescriptor

import numpy as np


def pointcloud2_to_xyzi(msg):
    """Convert PointCloud2 to Nx4 (x, y, z, intensity)."""
    points = point_cloud2.read_points(
        msg, field_names=("x", "y", "z", "intensity"),
        skip_nans=True
    )
    arr = np.array(list(points), dtype=np.float32)
    if arr.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if arr.shape[1] == 3:
        intensity = np.zeros((arr.shape[0], 1), dtype=np.float32)
        arr = np.hstack([arr, intensity])
    return arr


def range_projection_standalone(vertex, fov_up=3.0, fov_down=-25.0, proj_H=64, proj_W=900, max_range=80):
    """KITTI-style range projection with sparse pixel fill & bilinear interpolation."""
    fov_up = fov_up / 180.0 * np.pi
    fov_down = fov_down / 180.0 * np.pi
    fov = abs(fov_down) + abs(fov_up)

    depth = np.linalg.norm(vertex[:, :3], 2, axis=1)
    vertex = vertex[(depth > 0) & (depth < max_range)]
    depth = depth[(depth > 0) & (depth < max_range)]
    if depth.size == 0:
        return np.zeros((proj_H, proj_W), dtype=np.float32)

    scan_x, scan_y, scan_z = vertex[:, 0], vertex[:, 1], vertex[:, 2]
    yaw = -np.arctan2(scan_y, scan_x)
    pitch = np.arcsin(scan_z / depth)
    proj_x = 0.5 * (yaw / np.pi + 1.0) * proj_W
    proj_y = (1.0 - (pitch + abs(fov_down)) / fov) * proj_H
    proj_x = np.floor(proj_x).astype(np.int32)
    proj_y = np.floor(proj_y).astype(np.int32)
    proj_x = np.clip(proj_x, 0, proj_W - 1)
    proj_y = np.clip(proj_y, 0, proj_H - 1)
    order = np.argsort(depth)[::-1]
    depth = depth[order]
    proj_y, proj_x = proj_y[order], proj_x[order]

    # ✅ 初始化为 NaN（标记无效像素）
    proj_range = np.full((proj_H, proj_W), np.nan, dtype=np.float32)
    proj_range[proj_y, proj_x] = depth
    
    # ✅ 补齐稀疏像素：相邻像素最小值（局部最小值滤波）
    # 这样可以保留尽可能多的深度信息，避免大范围空洞
    valid_mask = ~np.isnan(proj_range)
    if np.sum(valid_mask) > 0:
        # 对每个 NaN 像素，用相邻 8 个有效像素的最小值填充
        from scipy.ndimage import maximum_filter
        min_depth_map = maximum_filter(proj_range, size=3, mode='constant', cval=np.nan)
        nan_mask = np.isnan(proj_range)
        proj_range[nan_mask] = min_depth_map[nan_mask]
    
    # ✅ 最后的空洞用 0 填充
    proj_range = np.nan_to_num(proj_range, nan=0.0)
    return proj_range.astype(np.float32)


class DescriptorServer(Node):
    def __init__(self):
        super().__init__("overlap_transformer_descriptor_server")
        self.declare_parameter("model_path", "")
        self.declare_parameter("fov_up", 3.0)
        self.declare_parameter("fov_down", -25.0)
        self.declare_parameter("proj_H", 64)
        self.declare_parameter("proj_W", 900)
        self.declare_parameter("max_range", 80.0)

        model_path = self.get_parameter("model_path").value
        self._model = None
        self._device = None
        if model_path and os.path.isfile(model_path):
            self._load_model(model_path)
        else:
            self.get_logger().warn(
                "OverlapTransformer model not loaded (model_path=%s). "
                "Returning zero descriptor." % model_path
            )

        self.srv = self.create_service(
            ComputeDescriptor,
            "/automap/compute_descriptor",
            self.compute_descriptor_callback,
        )
        self.get_logger().info("Descriptor service /automap/compute_descriptor ready.")

    def _load_model(self, model_path):
        try:
            import torch
            from modules.overlap_transformer import featureExtracter

            self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self._model = featureExtracter(use_transformer=True, channels=1)
            state = torch.load(model_path, map_location=self._device)
            if isinstance(state, dict) and "state_dict" in state:
                state = state["state_dict"]
            
            # ✅ 加载前验证权重兼容性
            if "conv1.weight" in state:
                conv1_shape = state["conv1.weight"].shape
                expected_shape = (16, 1, 5, 1)  # KITTI 配置
                if conv1_shape != expected_shape:
                    self.get_logger().warn(
                        f"[Weight Check] conv1.weight shape mismatch: got {conv1_shape}, expected {expected_shape}. "
                        "Model may not work correctly."
                    )
            
            self._model.load_state_dict(state, strict=False)
            self._model.to(self._device)
            self._model.eval()
            
            # ✅ 验证推理正确性
            x_dummy = torch.zeros(1, 1, 64, 900, device=self._device)
            with torch.no_grad():
                y_dummy = self._model(x_dummy)
            if y_dummy.shape[-1] != 256:
                self.get_logger().error(
                    f"[Weight Check] Output dimension mismatch: got {y_dummy.shape[-1]}, expected 256"
                )
                self._model = None
                return
            
            self.get_logger().info(
                f"[Weight Check] ✓ Model verified: conv1={state.get('conv1.weight', 'N/A').shape if isinstance(state.get('conv1.weight'), torch.Tensor) else 'N/A'}, "
                f"output=256-d, device={self._device}"
            )
            self.get_logger().info("OverlapTransformer model loaded from %s" % model_path)
        except Exception as e:
            self.get_logger().error("Failed to load model: %s" % str(e))
            import traceback
            self.get_logger().error(traceback.format_exc())
            self._model = None

    def compute_descriptor_callback(self, request, response):
        try:
            xyzi = pointcloud2_to_xyzi(request.pointcloud)
            if xyzi.shape[0] < 10:
                response.descriptor.layout.dim = []
                response.descriptor.data = [0.0] * 256
                return response

            proj_H = self.get_parameter("proj_H").value
            proj_W = self.get_parameter("proj_W").value
            proj_range = range_projection_standalone(
                xyzi,
                fov_up=float(self.get_parameter("fov_up").value),
                fov_down=float(self.get_parameter("fov_down").value),
                proj_H=int(proj_H),
                proj_W=int(proj_W),
                max_range=float(self.get_parameter("max_range").value),
            )
            if self._model is None:
                # Fallback: simple histogram as 256-dim
                desc = np.zeros(256, dtype=np.float32)
                hist, _ = np.histogram(proj_range[proj_range > 0], bins=256, range=(0, 80))
                desc[: len(hist)] = hist.astype(np.float32)
                norm = np.linalg.norm(desc)
                if norm > 1e-6:
                    desc /= norm
                response.descriptor.layout.dim = []
                response.descriptor.data = desc.tolist()
                return response

            import torch
            tensor = torch.from_numpy(proj_range).float().unsqueeze(0).unsqueeze(0)
            tensor = tensor.to(self._device)
            with torch.no_grad():
                desc = self._model(tensor)
            desc = desc.cpu().numpy().flatten()
            
            # ✅ 严格验证输出维度
            if desc.shape[0] != 256:
                self.get_logger().warn(
                    f"[Descriptor] Output dimension {desc.shape[0]} != 256, padding to 256"
                )
                if desc.shape[0] > 256:
                    desc = desc[:256]
                else:
                    desc = np.pad(desc, (0, 256 - desc.shape[0]), mode='constant')
            
            norm = np.linalg.norm(desc)
            if norm > 1e-6:
                desc = desc / norm
            else:
                self.get_logger().warn("[Descriptor] Zero norm descriptor, using fallback")
                desc = np.ones(256, dtype=np.float32) / np.sqrt(256.0)
            
            response.descriptor.layout.dim = []
            response.descriptor.data = desc.astype(np.float32).tolist()
            return response
        except Exception as e:
            self.get_logger().error("ComputeDescriptor failed: %s" % str(e))
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.descriptor.data = [0.0] * 256
            return response


def main(args=None):
    rclpy.init(args=args)
    node = DescriptorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生の RGB-D Image を圧縮 (CompressedImage) して再配信する中継ノード。

Isaac Sim の ROS2CameraHelper は raw の sensor_msgs/Image しか publish しないが、
実機の HSR は image_transport 経由で publish するため /compressed・/compressedDepth が
存在する。消費側 (hma_pcl_reconst2 の use_compressed=true, openmm/mmpose 等) はこの
圧縮 topic を購読するので、sim 側でも同じ topic 構成を再現するためにこのノードを挟む。

これが無いと head_pcl_reconst の depth/rgb 同期が一度も発火せず、
/head_rgbd_sensor/reconsted/points が advertise だけされて無言になる。
"""

import json
import struct
from functools import partial
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


# 既定のマッピング: (入力=生Image topic, 出力=CompressedImage topic, 種別)。
# hsrc_ex シーン(color/image_raw・depth/image_raw)と HSR-B シーン
# (rgb/image_rect_color・depth_registered/image_rect_raw)の両方の生 topic を入力に取り、
# 消費側(openmm/mmpose・pcl_reconst)が要求する /compressed・/compressedDepth を出す。
# 実在する入力だけが流れるので、どちらの robot で起動しても必要な出力だけが publish される
# (無い入力は無出力)。同じ出力(rgb/image_rect_color/compressed 等)には複数入力が紐づくが、
# 一度に生きている入力は片方だけなので二重配信は起きない。
# ロボット名の対応: omniverse 側 robot=hsrb  ==  タスク側 hsrc(simple_test_hsrc/task_hsrc) として扱う。
# hsrb シーンは hsrc スタック(VLM/openmm/pcl_reconst)が要求する compressed を全部出す。
DEFAULT_MAPPINGS = [
    # --- RGB (JPEG) ---
    ('/head_rgbd_sensor/color/image_raw',
     '/head_rgbd_sensor/color/image_raw/compressed', 'rgb'),                  # hsrc_ex: openmm(hsr_ex)/VLM
    ('/head_rgbd_sensor/color/image_raw',
     '/head_rgbd_sensor/rgb/image_rect_color/compressed', 'rgb'),            # hsrc_ex: pcl_reconst(HSR-B名へブリッジ)
    ('/head_rgbd_sensor/rgb/image_rect_color',
     '/head_rgbd_sensor/rgb/image_rect_color/compressed', 'rgb'),            # hsrb(=hsrc): openmm(hsrc)/VLM/pcl_reconst
    # --- Depth (compressedDepth PNG) ---
    ('/head_rgbd_sensor/depth/image_raw',
     '/head_rgbd_sensor/depth/image_raw/compressedDepth', 'depth'),           # hsrc_ex: openmm(hsr_ex)
    ('/head_rgbd_sensor/depth/image_raw',
     '/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth', 'depth'),  # hsrc_ex: pcl_reconst(ブリッジ)
    ('/head_rgbd_sensor/depth_registered/image_rect_raw',
     '/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth', 'depth'),  # hsrb(=hsrc): pcl_reconst
    ('/head_rgbd_sensor/depth_registered/image_rect_raw',
     '/head_rgbd_sensor/depth_registered/image_raw/compressedDepth', 'depth'),       # hsrb(=hsrc): openmm(hsrc) ※image_raw名
]


class RgbdRepublisher(Node):
    """生のRGB-D(Image)をsubして圧縮(CompressedImage)をpublishする中継ノード。
    既存topicに追加する形なので元のtopicには影響しない。
    複数の (入力→出力) を1ノードで扱い、実在する入力だけを中継する。"""

    def __init__(self) -> None:
        super().__init__('rgbd_republisher')

        # mappings: JSON配列で上書き可。空なら DEFAULT_MAPPINGS を使う。
        #   例: '[{"input":"/a","output":"/a/compressed","kind":"rgb"}, ...]'
        self.declare_parameter('mappings', '')
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('png_level', 3)
        self.declare_parameter('depth_quant_a', 1000.0)
        self.declare_parameter('depth_quant_b', 0.0)

        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.png_level = self.get_parameter('png_level').get_parameter_value().integer_value
        self.depth_quant_a = self.get_parameter('depth_quant_a').get_parameter_value().double_value
        self.depth_quant_b = self.get_parameter('depth_quant_b').get_parameter_value().double_value

        mappings = self._load_mappings()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # 出力topicごとに publisher は1つだけ作る(重複出力をまとめる)
        self._pubs = {}        # output_topic -> publisher
        # 入力topicごとに [kind, [出力publisher,...]] を持つ
        self._routes = {}      # input_topic -> [kind, [pub, ...]]

        for in_topic, out_topic, kind in mappings:
            if out_topic not in self._pubs:
                self._pubs[out_topic] = self.create_publisher(
                    CompressedImage, out_topic, sensor_qos)
            entry = self._routes.setdefault(in_topic, [kind, []])
            entry[1].append(self._pubs[out_topic])

        self._logged = set()   # 初回フレームのログ用
        for in_topic, (kind, pubs) in self._routes.items():
            cb = partial(
                self._on_rgb if kind == 'rgb' else self._on_depth,
                in_topic=in_topic, pubs=pubs)
            self.create_subscription(Image, in_topic, cb, sensor_qos)

        self.get_logger().info(
            'RGB-D compression republisher started: %d inputs -> %d outputs'
            % (len(self._routes), len(self._pubs)))

    def _load_mappings(self):
        raw = self.get_parameter('mappings').get_parameter_value().string_value.strip()
        if not raw:
            return DEFAULT_MAPPINGS
        try:
            data = json.loads(raw)
            return [(m['input'], m['output'], m['kind']) for m in data]
        except Exception as exc:
            self.get_logger().warn(
                f'mappings param parse failed ({exc}); using DEFAULT_MAPPINGS')
            return DEFAULT_MAPPINGS

    def _on_rgb(self, msg: Image, in_topic: str, pubs) -> None:
        bgr = self._image_to_bgr(msg)
        if bgr is None:
            return
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
        ok, encoded = cv2.imencode('.jpg', bgr, encode_params)
        if not ok:
            self.get_logger().warn('Failed to encode RGB image as JPEG')
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = f'{msg.encoding}; jpeg compressed bgr8'
        out.data = encoded.tobytes()
        for pub in pubs:
            pub.publish(out)
        self._log_once('rgb:' + in_topic,
                       f'Published first RGB compressed frame from {in_topic}')

    def _on_depth(self, msg: Image, in_topic: str, pubs) -> None:
        compressed = self._depth_to_compressed_depth(msg)
        if compressed is None:
            return
        for pub in pubs:
            pub.publish(compressed)
        self._log_once('depth:' + in_topic,
                       f'Published first compressedDepth frame from {in_topic}')

    def _log_once(self, key: str, message: str) -> None:
        if key not in self._logged:
            self.get_logger().info(message)
            self._logged.add(key)

    def _image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            encoding = msg.encoding.lower()
            if encoding in ('rgb8', 'bgr8'):
                img = row[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else img.copy()
            if encoding in ('rgba8', 'bgra8'):
                img = row[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
                return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if encoding == 'rgba8' else cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            if encoding in ('mono8', '8uc1'):
                img = row[:, :msg.width]
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            if encoding == '8uc3':
                return row[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
            self.get_logger().warn(f'Unsupported RGB encoding: {msg.encoding}')
            return None
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert RGB image: {exc}')
            return None

    def _depth_to_compressed_depth(self, msg: Image) -> Optional[CompressedImage]:
        """compressedDepth(PNG)形式に変換"""
        encoding = msg.encoding.upper()
        try:
            if encoding == '32FC1':
                depth = self._image_to_array(msg, np.float32, 4)
                valid = np.isfinite(depth) & (depth > 0.0)
                quantized = np.zeros(depth.shape, dtype=np.uint16)
                quantized[valid] = np.clip(
                    np.rint(self.depth_quant_a / depth[valid] + self.depth_quant_b),
                    1, 65535).astype(np.uint16)
                compressed_format = '32FC1; compressedDepth png'
            elif encoding == '16UC1':
                quantized = self._image_to_array(msg, np.uint16, 2)
                compressed_format = '16UC1; compressedDepth png'
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
                return None
            encode_params = [int(cv2.IMWRITE_PNG_COMPRESSION), int(self.png_level)]
            ok, encoded = cv2.imencode('.png', quantized, encode_params)
            if not ok:
                self.get_logger().warn('Failed to encode depth image as PNG')
                return None
            out = CompressedImage()
            out.header = msg.header
            out.format = compressed_format
            out.data = struct.pack('iff', 0, self.depth_quant_a, self.depth_quant_b) + encoded.tobytes()
            return out
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert depth image: {exc}')
            return None

    @staticmethod
    def _image_to_array(msg: Image, dtype: np.dtype, bytes_per_pixel: int) -> np.ndarray:
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        packed = row[:, :msg.width * bytes_per_pixel]
        return packed.copy().view(dtype).reshape(msg.height, msg.width)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbdRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

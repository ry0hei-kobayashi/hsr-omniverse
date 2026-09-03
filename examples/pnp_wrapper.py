#!/usr/bin/env python3
"""hsrb_pick_and_place を「腕主体」設定で起動するラッパー。

台車+腕の同時軌道はこのシミュ移植では追従誤差が出るため、
whole_body の base 移動ペナルティ (linear_weight / angular_weight) を
上げて、計画を腕主体 (台車ほぼ静止) にする。
事前に台車を対象の手前 0.5m 程度へ寄せておくこと。
"""
import rclpy
from hsrb_pick_and_place.hsrb_pick_and_place import HsrbPickAndPlace


def main():
    rclpy.init()
    node = HsrbPickAndPlace()
    # base をなるべく動かさない (既定 ~3/1 に対し大きな重み)
    node.whole_body.linear_weight = 100.0
    node.whole_body.angular_weight = 100.0
    node.get_logger().info(
        f'arm-first weights: linear={node.whole_body.linear_weight} '
        f'angular={node.whole_body.angular_weight}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

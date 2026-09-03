# examples/ — ロボットへ制御指示を送るサンプルコード

シミュレータ (Isaac Sim + HSR) に対して、ROS 2 経由で動作指令を送る側のコード。
実習で学生が書くコードもこの位置づけになる。

## 使い方 (共通)

ros2 コンテナの中で実行する:

```bash
make exec ros2                            # ros2 コンテナに入る
python3 /examples/grasp_only.py           # 例: 把持デモ
```

コンテナには `/examples` としてマウント済み (docker-compose.yml)。

## ファイル一覧

| ファイル | 内容 | 指令手段 |
| --- | --- | --- |
| `grasp_only.py` | 床のリンゴに手を伸ばして掴む (把持まで) | hsrb_interface (`whole_body`, `gripper`) |
| `drive_to.py` | 台車を odom 目標位置へ移動 `python3 drive_to.py <x> <y> <yaw>` | cmd_vel トピック + TF |
| `fake_grasp_pub.py` | 偽の物体検出を配信 (認識なしで pick and place を試す用) | GraspDetection トピック |
| `grasp_tf_broadcaster.py` | 把持点を `grasp_point` TF として可視化 | GraspDetection トピック → TF |
| `pnp_wrapper.py` | hsrb_pick_and_place を腕主体設定で起動するラッパー | (pick_and_place_example 用) |

## 指令の流れ

```
このフォルダのコード ──▶ ROS 2 トピック/アクション ──▶ scripts/hsr.py の
 (hsrb_interface or                                    アクションサーバ
  ros2 topic/action)                                   → Isaac Sim の関節を駆動
```

主な指令先:
- 台車: `/omni_base_controller/cmd_vel` (geometry_msgs/Twist)
- 腕:   `/arm_trajectory_controller/follow_joint_trajectory` (FollowJointTrajectory)
- 頭:   `/head_trajectory_controller/follow_joint_trajectory`
- 手:   `/gripper_controller/apply_force`, `/gripper_controller/grasp`
- 高レベル: hsrb_interface の `whole_body.move_end_effector_pose()` (MoveIt 計画つき)

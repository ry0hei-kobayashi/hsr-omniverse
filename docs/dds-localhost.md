# DDS を loopback に閉じる（LANへのマルチキャスト漏洩対策）

## 症状

Isaac Sim を起動している間だけ、同じLANにいる他の機器（特に Wi-Fi）の速度が激しく落ちる。
実測では家庭内の Wi-Fi が **500 Mbps → 50 Mbps** に低下し、Isaac を止めると回復した。

## 原因

ROS 2 の CycloneDDS が、ホスト内で完結すべき点群・画像を**物理NICからマルチキャストで送信**していた。

既定の `assets/cyclonedds.xml` は `<NetworkInterface autodetermine="true"/>` なので、
Cyclone は `lo` ではなく物理NIC（例 `enp3s0`）を選ぶ。さらに `<AllowMulticast>default</AllowMulticast>`
は有線NICでは実質 `true` になり、**ディスカバリだけでなくユーザデータもマルチキャスト送信**される。
Isaac も ROS ノードも同一ホスト上（compose は `network_mode: host`）なのに、全部 LAN へ出て行く。

実測値（1台あたり）:

| 項目 | 値 |
|---|---|
| `enp3s0` 送信 | 98.8 Mbps（常時） |
| 送信に占めるマルチキャスト比率 | 99.5% |
| パケットレート | 8,549 pps |
| 18時間の累積送信 | 302 GB |

家庭用ルータ/APは IGMP snooping が効かずこれを Wi-Fi へフラッディングし、
Wi-Fi はマルチキャストを最低基本レート・非集約で送るため AP の電波時間が飽和する。

## 対処

`assets/cyclonedds.localhost.xml` を使う。`lo` にバインドし、マルチキャストを完全に切り、
探索は localhost へのユニキャストで行う。既定の `assets/cyclonedds.xml` は残してあるので、
**環境変数で切り替える**（再ビルド不要）。

### 1. Isaac 側

```bash
cd ~/hsr-omniverse
make up localhost
```

`localhost` は `dev` と同じモディファイアで、アクションと並べて書く。付けると
`CYCLONEDDS_URI=file:///cyclonedds.localhost.xml` を Makefile が組み立てて渡す。
**URI を手打ちする必要はない。**

`ROS_DOMAIN_ID` を変える場合は同時に指定する（既定 26）:

```bash
make up localhost ROS_DOMAIN_ID=31
```

### 2. Apptainer 側（`hma2_ws` の学生環境）

```bash
cd ~/hma2_ws
bash 0_shell.sh
```

コンテナに入ったら、**引数に Isaac と同じ `ROS_DOMAIN_ID`** を渡して source する:

```bash
source ~/hsr-omniverse/scripts/isaac_localhost_mode.sh 31
```

成功すると次が出る:

```
✅  Isaac (localhost) mode
    ROS_DOMAIN_ID      = 31
    RMW_IMPLEMENTATION = rmw_cyclonedds_cpp
    CYCLONEDDS_URI     = file:///home/roboworks/hsr-omniverse/assets/cyclonedds.localhost.xml
```

このスクリプトが `5e_isaac_mode.sh` の実行（`ROBOT_NAME` 等を拾う）→ 3変数の上書き →
`ros2 daemon stop` までを正しい順序で行う。**`5e_isaac_mode.sh` を自分で source する必要はない。**

## 検証

```bash
# Apptainer 内: Isaac のトピックが見えること（90件前後）
ros2 topic list | wc -l
ros2 topic echo /joint_states --once

# ホスト側: 物理NICのマルチキャスト送信が止まったこと
A=$(ethtool -S enp3s0 | awk '/OutMCastOctets/{print $2}'); sleep 5
B=$(ethtool -S enp3s0 | awk '/OutMCastOctets/{print $2}')
echo "multicast TX: $(( (B-A)*8/5/1000000 )) Mbps"    # 期待値 0（対策前は 98）
```

## 自分の環境で漏洩を確認する方法

対策前後どちらでも使える診断コマンド。root 権限は不要。

```bash
# インタフェース別スループット（5秒間の差分）
awk 'NR>2{gsub(/:/," ");print $1,$2,$10}' /proc/net/dev > /tmp/a; sleep 5
awk 'NR>2{gsub(/:/," ");print $1,$2,$10}' /proc/net/dev > /tmp/b
join /tmp/a /tmp/b | awk '{rx=($4-$2)*8/5/1e6; tx=($5-$3)*8/5/1e6;
  if(rx>0.05||tx>0.05) printf "%-16s RX %8.2f Mbps  TX %8.2f Mbps\n",$1,rx,tx}'

# 物理NICのマルチキャスト送信だけを取り出す
ethtool -S enp3s0 | grep -E 'OutMCast|OutBCast'

# DDS がどのマルチキャストグループを物理NIC上で購読しているか
# 0100FFEF = 239.255.0.1 (CycloneDDS)、1266FFEF/0700FFEF = ign-transport (Gazebo)
cat /proc/net/igmp
```

Cyclone が実際にどのインタフェースを選んだかは、トレースを有効にすると分かる:

```bash
CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Tracing>
  <Verbosity>config</Verbosity><OutputFile>stderr</OutputFile>
</Tracing></Domain></CycloneDDS>' ros2 topic list 2>&1 | grep -E 'selected interfaces|ownip'
```

`selected interfaces: lo` / `ownip: udp/127.0.0.1` なら閉じている。
`enp3s0` や `192.168.x.x` が出たら LAN へ漏れている。

## ハマりどころ

- **`CYCLONEDDS_URI` を手打ちしない。`make up localhost` を使う。** 打ち間違えても
  Cyclone は `can't open configuration file ...` を 1 行出すだけで先へ進んでしまう。
  その後 `rmw_create_node: failed to create domain` → `rcl node's rmw handle is invalid`
  → Isaac 側は `hsr.py` の `rclpy.node.Node()` で例外 → **segfault**、という
  原因の分かりにくい落ち方をする（実例: `file:=///cyclonedds.localhot.xml` という
  `=` 混入 + `s` 抜けの 2 箇所ミスで両コンテナが exit 1）。
  `make up localhost` なら URI は Makefile が組み立て、不正な値は
  コンテナ起動前に `make` が弾く。
- **`5e_isaac_mode.sh` を自分で source しない。** あれは `ROS_DOMAIN_ID` を `config.toml` の
  値（26）に戻し、`CYCLONEDDS_URI` もマルチキャスト版に戻す。手で export した値が黙って巻き戻る。
- **`hma2_ws` 標準の `5a_default/cyclonedds.localhost_only.xml` は使えない。**
  あれは `autodetermine="true"` で物理NICにバインドするため、`lo` にいる Isaac とロケータが
  噛み合わず、自分の `/rosout` `/parameter_events` の 2 件しか見えない。
  **両側が同じ lo プロファイルであること**が条件。
- **`5e_isaac_mode.sh -s` / `--sim` は使わない。** `ROS_DOMAIN_ID` が 99 固定になり Isaac と噛み合わない。
- **`ros2 daemon stop` を忘れない。** デーモンが以前の `ROS_DOMAIN_ID` / RMW 設定を握ったままなので、
  環境変数を変えても `ros2 topic list` が空のままになる。上のスクリプトは自動で実行する。
- **Gazebo を併用する場合**は `GZ_IP=127.0.0.1` / `IGN_IP=127.0.0.1` も設定する
  （ign-transport は `239.255.0.7` 系へ別途マルチキャストする）。
- Apptainer は `--net` を付けずに起動するのでホストと同じネットワーク名前空間にいる。
  Docker 側も `network_mode: host` なので、両者は loopback を共有する。**これが本方式の前提。**

## 複数台で演習する場合

1台あたり 98.8 Mbps を出すので、20台を1つの1GbEスイッチに繋ぐと約 2 Gbps のマルチキャストが流れ、
IGMP snooping が無ければ各PCが他19台分（約 1.9 Gbps）を受信して 1GbE NIC の上限を超える。
さらに `ROS_DOMAIN_ID` が全台共通だと 20台のROSグラフが融合し、`/cmd_vel` 等が衝突して
**他人のロボットが動く**。

- 全台で本 localhost プロファイルを既定にする（これでLANトラフィックはほぼゼロになる）。
- 保険として `ROS_DOMAIN_ID` を台ごとにユニークにする（0〜101 の範囲）。**Isaac 側と Apptainer 側で同じ値**にすること。
- 演習LANは会場ネットから L2 分離する。Wi-Fi は演習セグメントに載せない。

台間で ROS 通信が必要な場合は localhost プロファイルは使えない。既定の `assets/cyclonedds.xml`
に戻したうえで `<AllowMulticast>spdp</AllowMulticast>`（探索のみマルチキャスト / データはユニキャスト）に変更し、
`<FragmentSize>` を 1400B 以下へ下げ、IGMP snooping 対応のマネージドスイッチを使うこと。

## 元に戻す

`localhost` を付けずに `make up` すれば従来どおり `assets/cyclonedds.xml`（マルチキャスト版）が使われる。
Apptainer 側も `source ~/hma2_ws/5e_isaac_mode.sh` に戻す。

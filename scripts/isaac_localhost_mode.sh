#!/bin/bash
#
# Apptainer (hma2_ws) 側から、loopback に閉じた Isaac Sim へ接続するための設定。
# hma2_ws のコンテナ内で source して使う。
#
#   source /home/roboworks/hsr-omniverse/scripts/isaac_localhost_mode.sh [ROS_DOMAIN_ID]
#
# 既定の ROS_DOMAIN_ID は 26。Isaac 側と必ず同じ値にすること:
#   Isaac 側:     ROS_DOMAIN_ID=31 CYCLONEDDS_URI=file:///cyclonedds.localhost.xml make up
#   Apptainer側:  source .../isaac_localhost_mode.sh 31
#
# なぜ専用スクリプトが要るのか:
#   - hma2_ws の 5e_isaac_mode.sh は ROS_DOMAIN_ID を config.toml の値 (26) に
#     戻し、CYCLONEDDS_URI もマルチキャスト版プロファイルに上書きする。
#     手で export した値が黙って巻き戻るので、必ず「source の後」に上書きする必要がある。
#   - hma2_ws 標準の 5a_default/cyclonedds.localhost_only.xml は使えない。
#     あれは autodetermine="true" で物理NIC (例: 192.168.0.222) にバインドするため、
#     lo (127.0.0.1) にバインドした Isaac とロケータが噛み合わず 1 トピックも見えない
#     (実測: 自分の /rosout /parameter_events の 2 件のみ)。
#   - ros2 daemon は以前の ROS_DOMAIN_ID / RMW 設定を握ったままなので、
#     止めないと環境変数を変えても topic list が空のままになる。

if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  echo "❌  ERROR: This script must be sourced." >&2
  echo "    source ${BASH_SOURCE[0]} [ROS_DOMAIN_ID]" >&2
  exit 1
fi

_ISAAC_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_ISAAC_PROFILE="$_ISAAC_REPO/assets/cyclonedds.localhost.xml"
_ISAAC_DOMAIN="${1:-${ROS_DOMAIN_ID:-26}}"

if [ ! -f "$_ISAAC_PROFILE" ]; then
  echo "❌  ERROR: プロファイルが見つかりません: $_ISAAC_PROFILE" >&2
  unset _ISAAC_REPO _ISAAC_PROFILE _ISAAC_DOMAIN
  return 1
fi

# 先に hma2_ws 側のモードスクリプトを通して ROBOT_NAME 等を拾う (コンテナ内のみ)。
# ここで ROS_DOMAIN_ID と CYCLONEDDS_URI が上書きされるので、この後で必ず入れ直す。
# 引数を明示的に空で渡すこと。省略すると呼び出し元の位置パラメータ ($1 = ドメインID) が
# そのまま渡り、向こうの case 文が "Unknown option" で早期 return してしまう。
if [ -n "${APPTAINER_NAME:-}" ] && [ -f "${COLCON_WORKSPACE:-}/5e_isaac_mode.sh" ]; then
  source "$COLCON_WORKSPACE/5e_isaac_mode.sh" "" || true
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$_ISAAC_PROFILE"
export ROS_DOMAIN_ID="$_ISAAC_DOMAIN"
# ROS_LOCALHOST_ONLY は CYCLONEDDS_URI と競合しうるので明示的に無効化しておく
export ROS_LOCALHOST_ONLY=0

# 古い設定を握ったデーモンを落とす (次の ros2 コマンドで自動的に再起動する)
ros2 daemon stop > /dev/null 2>&1 || true

echo "✅  Isaac (localhost) mode"
echo "    ROS_DOMAIN_ID      = $ROS_DOMAIN_ID   (Isaac 側と一致していること)"
echo "    RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "    CYCLONEDDS_URI     = $CYCLONEDDS_URI"

unset _ISAAC_REPO _ISAAC_PROFILE _ISAAC_DOMAIN

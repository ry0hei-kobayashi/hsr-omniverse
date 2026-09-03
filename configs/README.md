# configs/

シミュレーションの実行時設定を管理するディレクトリ。
**ここの YAML を編集するだけで、コードを触らずに見た目を変えられる**。

## ファイル一覧

| ファイル | 用途 |
|---|---|
| [`placement.yaml`](./placement.yaml) | シーン初期配置 (`robot` ロボット位置 / `objects` YCB物体配置 / `furniture` 家具 / `people` 人) をまとめた設定 |
| [`placement.compe.yaml`](./placement.compe.yaml) | 競技のObject Listと全Seed共通スポーン位置 |
| `placement.compe_seed1.yaml` 〜 `placement.compe_seed4.yaml` | `compe_seed=1..4` で選ぶ競技用固定配置 |
| [`dressing.yaml`](./dressing.yaml) | シーン演出 (床テクスチャ + 周囲背景画像 + 照明) のプリセット定義 |

## クイックスタート

### よく変更したい場面と編集箇所

| やりたいこと | 編集場所 |
|---|---|
| ロボットの初期位置を変える | [`placement.yaml`](./placement.yaml) の **`robot:`** (x, y, yaw[rad]) |
| 家具の上に物体を置く | `placement.yaml` の **`objects.placements:`** |
| 人 (立っている人 / 手を振る人) を出す | `placement.yaml` の **`people:`** → [下の手順](#人-people-を出す) |
| 部屋の見た目をプリセットごと切り替える (例: lab → office) | [`dressing.yaml`](./dressing.yaml) の **`defaults.preset:`** |
| 照明モードを切り替える (例: default → bright) | `dressing.yaml` の **`defaults.lighting:`** |
| 既存プリセットの値を微調整 (例: 天井灯を強くする) | `dressing.yaml` の **`lighting_presets.<name>:`** や **`dressing_presets.<name>:`** の中の数値 |
| 新しいテクスチャセットを追加する | `dressing_presets:` に dict エントリを追加 |
| 新しい照明モードを追加する | `lighting_presets:` に dict エントリを追加 |
| 床だけ / 背景だけにする | プリセットの該当値を `null` に |

### 編集後の反映方法

```bash
make down
make up
```

競技用配置を反映する場合:

```bash
make down
make up localhost compe_seed=1
```

`down` を先にやるのが大事(`up` だけだと既存コンテナが残っていて Python プロセスが再起動されず、YAML の変更が反映されないことがある)。

## 人 (`people`) を出す

`placement.yaml` の **`people:`** セクションで、部屋に人を立たせられる。
**既定では出さない** (`list: []`)。出したいときだけ有効にする。

### 出しかた

`placement.yaml` の `people:` を開いて、次の 2 つをする。

1. `list: []` の行を消す
2. その下の `# list:` から `# motion: stand_idle_wave_loop` までの行頭の `# ` を外す

そのまま `make down && make up` で、立っている人と手を振っている人が 1 人ずつ出る。
起動ログに `[people] spawned 2/2 people ...` と出れば成功。

### 書きかた

```yaml
people:
  list:
    - name: person_standing   # シーン内の名前 (/World/People/<name>)
      x: 2.4                  # 位置 (m, world 座標。robot:/objects: と同じ)
      y: 2.6
      yaw: 270                # 向き (★度。0=+X, 90=+Y, 180=-X, 270=-Y)
      motion: stand_idle_loop # 動き (下の表)
```

| `motion` | 動き |
|---|---|
| `stand_idle_loop` | まっすぐ立つ (その場で待つ) |
| `stand_idle_wave_loop` | 手を振る (立つ→振る→下ろす→立つ の繰り返し) |

この 2 つは人モデルごと [`../usd/isaac_offline/`](../usd/isaac_offline/) に同梱してあるので、
**ネットに繋がっていなくても動く**。`people_spawn.py` は `Sit` など他の名前も知っているが、
その USD は同梱していないのでネットが要る。

### 注意

- **`yaw` の単位は「度」**。`furniture:` と同じで、`robot:` の `yaw` (ラジアン) とは違う。
- **人は当たり判定を持たない**。見た目とアニメーションだけなので、ロボットは人をすり抜ける。
  経路をふさぐ用途には使えない。
- **手を上げっぱなしにしたいとき**は `people.loop_window:` (`placement.yaml` にコメントで用意)
  を使う。`stand_idle_wave_loop` には手を上げたままのクリップが無いので、再生する秒数を
  「手を振っている区間」だけに絞って上げっぱなしに見せる。
  ただし再生ヘッドはシーンに 1 本しかないので、**この区間指定は他の人にも同じように効く**。

## `dressing.yaml` の構造

3 つのトップレベルセクションがある:

```yaml
defaults:            # ← 起動時に使うプリセットの「名前」を指定
  preset: lab
  lighting: default

dressing_presets:    # ← 床テクスチャ + 背景画像 4 枚の組み合わせ集
  lab:
    floor_texture: /data/textures/floor.jpg
    backdrop_textures: [...]
  office:
    ...

lighting_presets:    # ← 照明強度・色温度などの組み合わせ集
  default:
    dome_intensity: 3000.0
    ceiling_intensity: 150000.0
    ...
  bright:
    ...
```

### `defaults` セクション
コードから `apply_lab_dressing()` を引数なしで呼んだとき、どのプリセットを使うかを指定する。

```yaml
defaults:
  preset: lab           # dressing_presets のキー名
  lighting: default     # lighting_presets のキー名
```

### `dressing_presets` セクション
床と壁(周囲背景画像)の組み合わせ。各エントリのキー:

| キー | 型 | 説明 |
|---|---|---|
| `floor_texture` | str / null | 床に貼る画像のパス。`null` で床作成スキップ |
| `backdrop_textures` | list / null | `[北, 南, 東, 西]` の 4 枚画像パス。`null` で背景作成スキップ |
| `floor_tile` | float | 床テクスチャのタイル繰り返し回数 (省略時 `4.0`) |
| `room_size` | float | 部屋一辺の長さ (m, 省略時 `15.0`) |
| `room_height` | float | 壁高さ (m, 省略時 `4.0`) |
| `floor_z` | float | 床の Z 座標 (m, 省略時 `0.002`) |

### `lighting_presets` セクション
照明強度・色温度などの組み合わせ。各エントリのキー:

| キー | 型 | 説明 |
|---|---|---|
| `dome_intensity` | float | `/World/EnvBox/Lights/Dome` (DomeLight) の強度 |
| `ceiling_intensity` | float | 天井 SphereLight 1 灯あたりの強度 |
| `ceiling_count` | int | 天井灯の数 (4 で四隅、5 で四隅 + 中央) |
| `ceiling_height` | float | 天井灯の Z 高さ (m) |
| `ceiling_color_temp` | float | 色温度 (K)。6500 = 昼白色、3000 = 電球色、5600 = デイライト |
| `enable_lighting` | bool | `false` にすると DomeLight + 天井灯を一切作成しない |
| `default_lights_intensity` | float | `/World/Light_1`, `/World/Light_2` (launch_isaacsim.py 既存ライト) の強度 |

## よくある操作の手順

### A. 既存プリセットの中身を変える

例: `default` 照明をもう少し明るくしたい。

`dressing.yaml` の `lighting_presets.default:` ブロックを編集:

```yaml
lighting_presets:
  default:
    dome_intensity: 5000.0          # 3000.0 → 5000.0
    ceiling_intensity: 250000.0     # 150000.0 → 250000.0
    ...
```

保存 → `make down && make up`。

### B. プリセットを切り替える

例: 一時的に `studio` で確認したい。

`dressing.yaml` の冒頭:

```yaml
defaults:
  preset: lab
  lighting: studio    # ← 切り替える
```

保存 → `make down && make up`。

### C. 新しいテクスチャセットを追加

1. ホスト側にテクスチャ画像を配置:
   ```
   ~/datasets/OfficeTextures/
   ├── carpet.jpg
   ├── office_n.jpg
   ├── office_s.jpg
   ├── office_e.jpg
   └── office_w.jpg
   ```

2. `env_docker/docker-compose.yml` の `isaacsim:` サービスの `volumes:` に追加:
   ```yaml
   - ${HOME}/datasets/OfficeTextures:/data/OfficeTextures:ro
   ```

3. `dressing.yaml` の `dressing_presets:` に新エントリ:
   ```yaml
   office:
     floor_texture: /data/OfficeTextures/carpet.jpg
     backdrop_textures:
       - /data/OfficeTextures/office_n.jpg
       - /data/OfficeTextures/office_s.jpg
       - /data/OfficeTextures/office_e.jpg
       - /data/OfficeTextures/office_w.jpg
     floor_tile: 6.0
   ```

4. 切り替え:
   ```yaml
   defaults:
     preset: office     # ← lab → office
   ```

5. `make down && make up`

### D. 新しい照明モードを追加

例: スタジオよりさらに明るいモードを追加。

```yaml
lighting_presets:
  ultra:
    dome_intensity: 30000.0
    ceiling_intensity: 3000000.0
    ceiling_count: 5
    ceiling_color_temp: 5600.0
    default_lights_intensity: 1000000.0
```

切り替えは `defaults.lighting: ultra` で。

### E. プリセット間で違いを見比べたい

`defaults.lighting:` を `dim` ↔ `bright` ↔ `studio` の順に変えて、`make down && make up` を 3 回繰り返せばどう変わるか比較できる。

## 注意事項

### ⚠️ YAML "Norway problem" — `off`, `on`, `yes`, `no` はクオートする

YAML 1.1 仕様では以下のキーワードがブール値として解釈される:

```
off, Off, OFF, on, On, ON, yes, Yes, YES, no, No, NO, true, True, TRUE, false, False, FALSE
```

つまり下のように書くと、`lighting` の値は文字列 `"off"` ではなくブール値 `False` に化ける:

```yaml
defaults:
  lighting: off       # ← NG: False に化ける
```

文字列として扱いたい場合はクオート必須:

```yaml
defaults:
  lighting: "off"     # ← OK: 文字列 "off"
```

同じくプリセット名側も:

```yaml
lighting_presets:
  "off":              # ← クオート
    enable_lighting: false
```

### ⚠️ 指数表記 (1.5e5 等) は文字列扱いされることがある

YAML 1.1 では `6e5` のような表記がパターンによっては文字列扱いになることがある。安全のため、float 値は明示的に小数点付き(普通の十進数)で書く:

```yaml
ceiling_intensity: 600000.0    # ← OK
# ceiling_intensity: 6e5       # ← NG: 環境によっては文字列扱い
```

(`dressing_presets.py` 側で float 化リカバリーは入れているが、書く側でも明示しておくのが無難)

### ⚠️ 変更を反映するには Python プロセス再起動が必要

bind mount でホストの YAML 編集は即時にコンテナから見えるが、`dressing_presets.py` が YAML を読み込むのは **Python の `import` 時 1 回だけ**。
したがって YAML を編集しただけでは反映されず、コンテナ(= Python プロセス)を再起動する必要がある:

```bash
make down
make up
```

### ⚠️ テクスチャパスはコンテナ内パス

`floor_texture` などに書くパスはコンテナ内部のパス。ホスト側のパスではない。
デフォルトでは [`./textures/`](./textures/) を docker-compose が `/data/textures` に bind mount しているので、`/data/textures/...` を指定すれば repo 内のサンプルがそのまま使える。
本番テクスチャの差し替え方は [`./textures/README.md`](./textures/README.md) を参照。

## 一覧: 同梱されているプリセット

### `dressing_presets` (テクスチャ)

| 名前 | 説明 |
|---|---|
| `rc26_venue` | 会場風(白壁 + 木目床)。**既定プリセット** |
| `lab` | ラボ風(`/data/textures/` 配下の floor, wall_1〜4) |
| `floor_only` | 床のみ、背景なし |
| `backdrop_only` | 背景のみ、床なし |

### `lighting_presets` (照明)

| 名前 | 強度感 | 用途例 |
|---|---|---|
| `default` | 標準 | 一般的な作業 |
| `bright` | default の 約3倍 | 撮影・スクリーンショット用 (明るめ) |
| `studio` | default の 10 倍 | スタジオ級・オーバー気味 |
| `dim` | default の 1/10 | 夜の部屋 |
| `warm` | default + 電球色 | 暖色系の屋内 |
| `"off"` | 全部消灯 | デバッグ・真っ暗確認 |

## 関連ファイル

`placement.yaml` を読み込む側:
- [`../scripts/launch_isaacsim.py`](../scripts/launch_isaacsim.py) — `robot:` セクションを読んで HSR を配置
- [`../scripts/object_placement.py`](../scripts/object_placement.py) — `objects:` セクションを読んで物体を配置
- [`../scripts/people_spawn.py`](../scripts/people_spawn.py) — `people:` セクションを読んで人を配置
- [`../scripts/furniture_spawn.py`](../scripts/furniture_spawn.py) — `furniture:` セクションを読んで本物の机・椅子 USD を配置

`dressing.yaml` を読み込む側:
- [`../scripts/dressing_presets.py`](../scripts/dressing_presets.py) — この YAML を読み込む Python ローダー
- [`../scripts/construct_environment.py`](../scripts/construct_environment.py) — `apply_lab_dressing()` 関数の実装
- [`../scene_dressing/`](../scene_dressing/) — 実際の床・背景・照明を作る Isaac Sim パッケージ

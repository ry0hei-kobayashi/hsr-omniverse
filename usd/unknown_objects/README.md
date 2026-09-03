# Unknown objects

競技用の未知物体を `configs/placement.yaml` からスポーンするためのローカル USD
アセットです。各ディレクトリは `model.usd` を入口にし、設定ではディレクトリ名を
`object:` に指定します。

```yaml
- object: logitech_m310_mouse
  x: 2.0
  y: 1.0
- object: coca_cola_bottle
  x: 2.3
  y: 1.0
- object: hamburger
  x: 2.6
  y: 1.0
- object: dualshock
  x: 2.9
  y: 1.0
```

`model.usd` は、USDZ から展開した `source.usdc` と同じディレクトリ内のテクスチャを
参照し、Y-up から Z-up への回転と実寸へのスケール補正を行います。

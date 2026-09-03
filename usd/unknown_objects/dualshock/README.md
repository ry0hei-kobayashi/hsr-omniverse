# DualShock / DualSense controller

`model.usd` is the Isaac Sim entry point. The supplied source is named
`dual+sense.usdc`; the directory remains `dualshock` so placement files can use
`object: dualshock`.

The wrapper disables a stray background plane, lays the controller flat, and
scales it to approximately 0.160 x 0.153 x 0.067 m. Only
`color_0A0A0A.exr` was supplied with the source. The other referenced image
files were missing, so small neutral PBR fallback textures are included to keep
all USD dependencies resolvable in offline Isaac Sim runs.

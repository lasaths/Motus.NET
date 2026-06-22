# Safety

## Motus v1 is not a robot controller

Motus provides:

- Joint-space motion **planning**
- Trajectory **validation**
- Rhino **preview**
- **Export** (JSON, CSV, joint lists)

Motus does **not**:

- Open network connections to robot controllers
- Send RTDE, RSI, or vendor motion commands
- Bypass safety interlocks or speed limits on hardware

## Preset disclaimer

Robot presets are **planning and visualization defaults**. They are not certified for any specific physical installation. Users must independently verify:

- Joint limits and soft limits on the real controller
- Tool center point (TCP) and base frame calibration
- Payload and center of gravity
- Safety-rated reduced mode, zones, and stop categories
- Mastering, encoder calibration, and cell layout

## Future execution features

Any future execution-related capability must be:

- Explicitly opt-in
- Dry-run / export-first by default
- Safety-gated with clear UI warnings
- Separate from the default planning-only plugin path

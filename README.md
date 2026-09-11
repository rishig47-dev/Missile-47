# Missile Trajectory Simulation & Animation (MATLAB)

A set of self-contained MATLAB scripts that simulate and visualize a generic
guided projectile's flight using a 3-DOF point-mass model. These are
teaching/visualization tools built on simplified, publicly documented
physics — not a real weapon system design.

## Requirements

- MATLAB R2016b or later (uses local functions in scripts)
- No toolboxes required (all math is implemented from scratch)
- For video export in the advanced animation script: base MATLAB's
  `VideoWriter` with the `MPEG-4` profile (Windows/Mac; on Linux use
  `'Motion JPEG AVI'` instead if `MPEG-4` is unavailable)

## Files

### 1. `missile_trajectory_simulation.m`
The core simulation. Run this first to understand the model and see full
results.

- US Standard Atmosphere (1976) density/temperature/speed-of-sound model
- Mach-dependent drag coefficient (lookup table)
- Multi-phase thrust profile (boost → sustain → coast) with mass depletion
- Altitude-varying (inverse-square) gravity
- Constant wind + random gusts
- Proportional Navigation (PN) guidance against a moving target
- RK4 fixed-step integration
- Outputs: console summary, 3D trajectory plot, altitude/speed/Mach/guidance
  subplots, ground track, and a basic animation

**Run it as-is** — all parameters are set in the `cfg` struct at the top.

### 2. `missile_trajectory_advanced_animation.m`
A cinematic, feature-rich animation built on the same physics model.
Standalone — does not depend on the other files.

- Missile body oriented along the velocity vector (not just a dot marker)
- Exhaust plume particles that spawn during powered flight and fade with age
- Motion trail colored by instantaneous speed (with colorbar)
- Dynamic chase camera that follows/banks with the missile
- Ground plane grid + shadow marker under the missile
- Live HUD overlay (time, altitude, speed, Mach, range-to-target, guidance g)
- Synced mini-plots (altitude & Mach vs time) with a moving cursor
- Optional MP4 export (`anim.export_video = true`)

Tune animation behavior via the `anim` struct (playback speed, trail length,
plume lifetime, video export, missile glyph size).

**Note:** particle transparency (`AlphaData` on `scatter3`) behaves slightly
differently across MATLAB versions. If plume fading doesn't render on your
version, the particles will still show but without the fade effect.

### 3. `missile_point_to_point_animation.m`
A simplified, robust animation for a straightforward "goes from point A to
point B" visualization — no HUD, no particles, no camera tricks.

- Set `P_start` and `P_end` (each `[X, Y, Altitude]` in meters) at the top
- The same physics model (gravity, drag, thrust, PN guidance) flies the
  missile from start toward the end point
- A single marker animates smoothly along the resulting path over a fixed
  playback duration (`playback_seconds`, default 6s), leaving a trail

This is the best starting point if you just want to see motion from one
location to another without digging into the full feature set.

## Quick Start

```matlab
% See the full physics model and static plots:
missile_trajectory_simulation

% See the advanced cinematic animation:
missile_trajectory_advanced_animation

% See a simple point-to-point animation:
missile_point_to_point_animation
```

## Customizing

All three scripts expose their tunable parameters near the top of the file
in plain structs (`cfg`, `anim`) — no need to touch the dynamics code unless
you want to change the underlying physics. Common things to adjust:

| Parameter | Location | Effect |
|---|---|---|
| `cfg.thrust_profile` | all scripts | boost/sustain/coast thrust levels and timing |
| `cfg.mach_table` / `cfg.cd_table` | all scripts | drag coefficient vs. Mach number |
| `cfg.N_gain` | scripts 1 & 2 | Proportional Navigation aggressiveness |
| `cfg.target_pos0_m` / `cfg.target_vel_ms` | scripts 1 & 2 | target starting position/velocity |
| `P_start` / `P_end` | script 3 | launch and destination points |
| `anim.export_video` | script 2 | save animation to MP4 instead of/besides live playback |

## Known Limitations

- 3-DOF point-mass model only (no attitude/rotational dynamics — the
  missile body is drawn oriented to velocity, but there's no full 6-DOF
  rigid-body simulation). Ask if you'd like a 6-DOF extension.
- Aerodynamic and atmospheric data are simplified/illustrative, not tied to
  any specific real vehicle.
- PN guidance implementation is for demonstration, not flight-certified.

## Troubleshooting

- **`Undefined function or variable`**: make sure you're running the whole
  file (local functions must stay at the bottom of the same `.m` file).
- **Video export fails**: try switching the `VideoWriter` profile from
  `'MPEG-4'` to `'Motion JPEG AVI'` (common on Linux).
- **Animation runs but looks choppy**: lower `anim.playback_fps` or increase
  `cfg.dt` slightly to reduce the data resampled per frame.

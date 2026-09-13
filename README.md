# KineRTL — Hardware-Accelerated Robot Arm Toolkit

An FPGA library for robot arm math: kinematics, trajectory generation, and everything a real-time motion controller needs, implemented as synthesizable VHDL instead of software. The goal is a growing collection of self-contained, hardware-verified building blocks that can be dropped into a robotics pipeline wherever a CPU is too slow, too unpredictable, or simply too busy to be trusted with the timing.

The project is designed to run alongside ROS2, offloading the parts of a control loop that need to happen at fixed, guaranteed intervals — like "where is the tool right now" or "what should the next setpoint be" — onto programmable logic, where the answer always arrives in the same number of clock cycles no matter what else the system is doing.

Everything in this document is either in progress or on the roadmap, listed here so the shape of the finished library is clear from the start.

## Why do this on an FPGA at all?

A robot's control loop runs on a strict schedule, but the math behind it — trigonometry, matrix algebra, numerical iteration — is exactly the kind of workload a general-purpose CPU handles inconsistently: cache misses, OS scheduling, and shared load with everything else the system is doing all introduce jitter. None of that exists in dedicated hardware. A circuit built to compute one thing takes the same number of clock cycles every single time, which is precisely the property a real-time system needs and software struggles to guarantee.

## Repository structure

```
ros2-fpga/
├── common/                      -- shared, robot-agnostic building blocks
│   ├── fixed_point_pkg.vhd      -- Qx.y fixed-point types and constants
│   ├── cordic/
│   │   ├── cordic.vhd           -- sine/cosine core
│   │   ├── cordic_atan2.vhd     -- (planned) angle-from-vector core
│   │   └── cordic_sqrt.vhd      -- (planned) square-root core
│   └── matrix_ops/
│       ├── mult_m.vhd           -- 4x4 fixed-point matrix multiplier
│       ├── mat_transpose.vhd    -- (planned)
│       └── mat_inverse.vhd      -- (planned, needed for numerical IK / Jacobians)
│
├── kinematics/
│   ├── forward/                 -- direct kinematics (IMPLEMENTED)
│   │   ├── robot_config_pkg.vhd -- per-robot DH parameter table
│   │   ├── htm_pkg.vhd
│   │   ├── direct_kinematics.vhd
│   │   └── direct_kinematics_tb.vhd
│   ├── inverse/
│   │   ├── analytical/          -- (planned) closed-form geometric solvers
│   │   └── numerical/           -- (planned) Jacobian-based iterative solver
│   └── jacobian/                -- (planned) geometric Jacobian computation
│
├── trajectory/
│   ├── joint_space/             -- (planned) per-joint motion profiles
│   └── cartesian_space/         -- (planned) tool-frame path interpolation
│
├── dynamics/                    -- (long-term) torque/force estimation
│
├── safety/                      -- (planned) joint limit & workspace guards
│
├── interfaces/                  -- (planned) AXI-Lite / UART bridges for ROS2
│
└── README.md
```

Each module lives with its own testbench next to it, and only depends on `common/` and, where relevant, a robot-specific configuration package. The intent is that any module can be pulled out and reused on its own.

## Functional modules

### Forward kinematics — Implemented

**What it does.** Given the current angle of every joint, computes the exact position and orientation of the robot's tool as a 4×4 homogeneous transformation matrix.

**How.** One CORDIC circuit per joint computes sine and cosine of every joint angle in parallel, without any floating-point hardware. Those results are combined with the robot's physical dimensions (arm lengths, offsets, and twist angles, following the standard Denavit-Hartenberg convention) to build one small transformation matrix per joint. A dedicated 4×4 matrix multiplier then chains all of those matrices together, one joint at a time, into the final pose. Every value in the design — angles, dimensions, matrix entries — is stored as fixed-point (f.e Q8.24), not floating point, which is what keeps the hardware small and fast.

**Results.** Simulated at 100 MHz against the included 6-joint configuration:

| Stage | What happens | Approx. time |
|---|---|---|
| Sine/cosine for all joints | All CORDIC circuits run in parallel | ~270 ns |
| Building the joint matrices | One matrix assembled per joint | ~10 ns |
| Chaining the matrices together | 6 joints, multiplied one after another | ~495 ns |
| Producing the final result | Widening and presenting the output | ~10 ns |
| **Total, start to finished pose** | | **~785 ns** |

That's over a million full kinematic solutions per second if run back-to-back, with identical latency every time. Positional resolution is set by the Q8.24 format at 2⁻²⁴ (roughly 6 × 10⁻⁸ of whatever unit the robot's dimensions are expressed in — sub-micrometer for the example robot included here), and the CORDIC stage keeps its own angular error below about 2⁻²⁶ radians, small enough to disappear into that same rounding. See `kinematics/forward/` for the full writeup, DH table format, and how to point the design at a different robot.

### Inverse kinematics — Planned

**What it will do.** The reverse problem: given a desired tool position and orientation, work out what joint angles produce it.

**How it's planned to work.** Two complementary approaches:
- **Analytical (closed-form).** For robots whose geometry allows it (e.g. a spherical wrist), the joint angles can be derived directly from trigonometric identities — fast, exact, and cheap in hardware, but specific to one kinematic structure at a time.
- **Numerical (iterative).** For arbitrary geometries, an iterative Jacobian-based method (Newton-Raphson or damped least squares) that starts from a guess and refines it toward the target pose over a handful of pipelined iterations. This is the more general solution and the one that will make the library usable for robots that don't have a closed-form solution.

Both will build directly on top of the existing forward kinematics and matrix building blocks.

### Jacobian computation — Planned

**What it will do.** Compute the geometric Jacobian matrix relating joint velocities to the tool's linear and angular velocity — the basis for velocity control, singularity detection, and the numerical IK solver above.

**How it's planned to work.** Derived directly from the intermediate transformation matrices already produced while computing forward kinematics, so this module is designed to reuse that pipeline's output rather than recompute it from scratch. A condition-number or determinant check on the resulting Jacobian will also provide basic singularity detection.

### Trajectory generation — Planned

**What it will do.** Turn a start pose, an end pose, and a duration into a smooth sequence of intermediate setpoints, either per joint or along a Cartesian path.

**How it's planned to work.**
- **Joint space:** classic motion profiles — trapezoidal velocity and quintic polynomial interpolation — computed incrementally, one setpoint per control cycle, without needing to store the whole trajectory.
- **Cartesian space:** linear interpolation of position combined with spherical interpolation (SLERP) of orientation, so the tool moves along a straight, predictable path in space rather than an arbitrary curve in joint space.

### Dynamics — Long-term / exploratory

**What it will do.** Estimate the joint torques required to achieve a given motion, accounting for the robot's mass distribution and the effects of gravity, inertia, and coupling between joints.

**How it's planned to work.** A pipelined recursive Newton-Euler formulation, computed link by link in a manner similar to how forward kinematics chains matrices together. This is significantly more arithmetic than anything else in the library and is being treated as a longer-term goal.

### Safety and limit checking — Planned

**What it will do.** Continuously check computed joint angles and tool positions against configured joint limits and workspace boundaries, flagging violations before they reach the physical robot.

**How it's planned to work.** Simple comparator logic running in parallel with the kinematics pipeline, so it costs no extra latency and can gate motion commands directly in hardware rather than relying on a software watchdog.

### ROS2 / host interface — Planned

**What it will do.** Expose these hardware modules to a ROS2 node running on a connected host, so joint states and setpoints can flow between software and the FPGA with minimal overhead.

**How it's planned to work.** An AXI-Lite register interface (for SoC platforms like Zynq) and a simpler UART-based bridge (for standalone FPGA boards), both wrapping the same underlying modules so the choice of interface doesn't affect the math.

### Common math core

**Status:** partially implemented (CORDIC sine/cosine, 4×4 matrix multiply), with matrix inversion/transpose, `atan2`, and fixed-point square root planned as they're needed by the modules above. The intent is for every higher-level module to be built from this shared, independently-tested set of primitives rather than reimplementing fixed-point arithmetic each time.

## Status overview

| Module | Status |
|---|---|
| Forward kinematics | Implemented & verified |
| CORDIC (sin/cos) | Implemented & verified |
| 4×4 matrix multiplier | Implemented & verified |
| Jacobian computation | Planned |
| Inverse kinematics (analytical) | Planned |
| Inverse kinematics (numerical) | Planned |
| Trajectory generation (joint space) | Planned |
| Trajectory generation (Cartesian space) | Planned |
| Safety / limit checking | Planned |
| ROS2 / host interface | Planned |
| Dynamics (Newton-Euler) | Long-term |

## Building and simulating

The project is developed and verified with [GHDL](https://ghdl.github.io/ghdl/), a free VHDL simulator. Nothing here depends on vendor-specific primitives, so it should also drop into Vivado or Quartus without changes. Every module ships next to its own testbench, following the same pattern:

```bash
cd kinematics/forward
ghdl -a robot_config_pkg.vhd
ghdl -a ../../common/cordic/cordic.vhd
ghdl -a htm_pkg.vhd
ghdl -a ../../common/matrix_ops/mult_m.vhd
ghdl -a direct_kinematics.vhd
ghdl -a direct_kinematics_tb.vhd
ghdl -e direct_kinematics_tb
ghdl -r direct_kinematics_tb --vcd=ondas.vcd --stop-time=50000ns
```

Open the resulting `.vcd` file with GTKWave (or any waveform viewer) to step through the calculation. Testbenches expose extra debug signals — like which joint is currently being processed — specifically to make that easy to follow.

## Adapting forward kinematics to a different robot

Everything specific to one robot lives in `kinematics/forward/robot_config_pkg.vhd`.

1. **Update the DH table (`ROBOT_ROM`).** Replace it with the new robot's arm lengths and offsets, scaled into the same fixed-point format as the rest of the design (multiply the real value by 2²⁴ and round). The number of joints is derived automatically from the table; nothing else needs manual resizing.

2. **Check the twist angles.** Only the four twist angles common to most industrial robots — 0°, 90°, 180°, -90° — are supported out of the box, since their sine and cosine are known ahead of time and hardcoded to save hardware. A different twist angle requires adding a case to `alpha_enum` and its corresponding branch in `direct_kinematics.vhd`, plus a CORDIC evaluation to supply its sine and cosine at runtime.

3. **Reconsider the fixed-point format if needed.** `TOTAL_WIDTH` and `FRAC_WIDTH` control range and precision. A much larger robot may need more integer bits; a robot needing finer resolution can trade some integer range for extra fractional bits.

4. **Update the testbench** with the joint configurations you want to verify.

5. **Tune `ITERATIONS_CORDIC`** if you want to trade a little precision for a shorter pipeline, or vice versa.

## Design notes

- **The CORDIC lookup table is generated, not typed in by hand.** It's computed by a VHDL function at compile time, so changing the iteration count or fixed-point format regenerates correct values automatically.
- **Rounding is symmetric, not truncated.** Every rescale back to the standard word size adds half an LSB before shifting, which keeps small rounding errors from consistently biasing results in one direction across several chained multiplications.

## Contributing

The modules marked "Planned" above are open territory. If you're picking one up, keep the same conventions the existing modules follow: a self-contained VHDL entity with its own testbench, fixed-point arithmetic throughout, and no vendor-specific primitives, so it stays portable across toolchains.

## License

Copyright (c) 2026 Jose Segura Montes. See source file headers for licensing terms.

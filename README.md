# ros2-fpga — FPGA-Accelerated N-DOF Robot Direct Kinematics

A parameterizable VHDL implementation of the Direct Kinematics problem for an N-DOF industrial manipulator, designed to run entirely in hardware. Joint angles go in, the homogeneous transformation matrix of the end effector comes out — no soft-core CPU, no floating point unit, no external math library. Just a pipelined datapath built from CORDIC engines and a custom fixed-point matrix multiplier.

The project was built as the compute core for a larger ROS2 + FPGA robotics pipeline, where offloading the forward kinematics chain to programmable logic removes it from the real-time constraints of a general-purpose processor.

## Why

Direct kinematics is a small amount of math (a handful of sines, cosines, and 4×4 matrix products) but it sits on the critical path of any real-time control loop. Running it on a CPU means paying for an OS scheduler, cache misses, and a general-purpose ALU that wasn't built for this. Running it on an FPGA means the latency is deterministic, known at compile time, and independent of whatever else the system is doing.

This design targets that trade-off directly: everything is fixed-point, everything is pipelined, and the amount of hardware instantiated scales automatically with the number of joints of the robot being modeled.

## Architecture

The system is split into three cooperating modules, orchestrated by a top-level FSM.

```
                     ┌─────────────────────────────────────────┐
                     │           direct_kinematics              │
                     │                                           │
   theta_in[0..N-1]  │   ┌──────────┐  ┌──────────┐             │
   ────────────────► │   │ CORDIC 0 │  │ CORDIC 1 │  ...        │
                     │   └────┬─────┘  └────┬─────┘             │
                     │        │  sin/cos     │                  │
                     │        ▼              ▼                  │
                     │   ┌─────────────────────────────┐        │
                     │   │  DH matrix assembly (MULT)   │        │
                     │   └──────────────┬───────────────┘        │
                     │                  ▼                        │
                     │   ┌─────────────────────────────┐        │
                     │   │  mult_m (sequential 4x4      │        │
   t_matrix ◄─────── │   │  accumulator, one link/iter) │        │
   done     ◄─────── │   └─────────────────────────────┘        │
                     └─────────────────────────────────────────┘
```

**1. Parallel CORDIC array.** One `cordic` core is instantiated per degree of freedom via a `generate` loop, so all joint angles are converted to sine/cosine simultaneously instead of one at a time. Each core runs the classic rotation-mode CORDIC algorithm with a compile-time generated arctangent lookup table (computed with `math_real` at elaboration time, not stored as a literal array).

**2. DH matrix assembly.** Once every CORDIC core reports `done`, the FSM builds the individual 4×4 Denavit-Hartenberg transformation matrix for each link in a single cycle, using the corresponding `sin`/`cos` pair together with the link's `a`, `d`, and `alpha` parameters pulled from a ROM. The four possible values of `alpha` (0°, 90°, 180°, -90°) are hardcoded as an enumerated type, which avoids computing sine/cosine of the twist angle in hardware — it's known at synthesis time.

**3. Sequential matrix multiplier.** `mult_m` is a small pipelined 4×4 matrix multiplier (multiply → partial sum → partial sum → writeback). The top-level FSM feeds it one link matrix per iteration, accumulating the running product into `t_matrix_reg` until all `DOF` links have been chained together. This is the only strictly sequential part of the pipeline, since each accumulation step depends on the previous one.

### Fixed-point format

All arithmetic uses a Qx.y signed fixed-point representation defined once in `robot_config_pkg`:

| Constant | Value | Meaning |
|---|---|---|
| `TOTAL_WIDTH` | 32 bits | word size for angles, DH parameters, and matrix elements |
| `FRAC_WIDTH` | 24 bits | fractional bits (Q8.24) |
| `fp_type` | `signed(31 downto 0)` | standard fixed-point word |
| `fp_mult_type` | `signed(63 downto 0)` | double-width word for raw multiplier outputs |

Every multiplication produces a `fp_mult_type` result, which is later rescaled back down with a symmetric round-to-nearest (`+ FP_HALF_MULT` before the shift) rather than a truncation, to keep rounding error from compounding across six chained matrix products.

### Robot configuration

The kinematic chain is described as a table of standard DH parameters in `robot_config_pkg.vhd`:

```vhdl
constant ROBOT_ROM : dh_rom_type := (
    0 => (a => ..., d => ..., alpha => A_90),
    1 => (a => ..., d => ..., alpha => A_0),
    ...
);
constant DOF : integer := ROBOT_ROM'length;
```

`DOF` is derived automatically from the length of the array, and every port and internal structure in `direct_kinematics` (`theta_in`, the CORDIC array, `matrix_array_dk`, etc.) is sized off that constant. Retargeting the design to a different robot means editing this one table — no changes to the datapath are required as long as the number of joints and their twist angles fit the existing types.

## File structure

```
.
├── cordic/
│   ├── cordic.vhd            -- single-joint CORDIC sine/cosine core
│   └── cordic_tb.vhd         -- standalone testbench
├── mult_matrix/
│   ├── mult_m.vhd            -- pipelined 4x4 fixed-point matrix multiplier
│   └── mult_m_tb.vhd         -- standalone testbench
├── direct_kinematics/
│   ├── robot_config_pkg.vhd  -- fixed-point format + DH parameter ROM
│   ├── htm_pkg.vhd           -- shared matrix/array type definitions
│   ├── direct_kinematics.vhd -- top-level FSM tying everything together
│   └── direct_kinematics_tb.vhd
└── README.md
```

## Building and simulating

The project is developed and verified with [GHDL](https://ghdl.github.io/ghdl/). No vendor-specific primitives are used, so it should port to Vivado or Quartus without modification.

### CORDIC core

```bash
cd cordic
ghdl -a ../direct_kinematics/robot_config_pkg.vhd
ghdl -a cordic.vhd
ghdl -a cordic_tb.vhd
ghdl -e cordic_tb
ghdl -r cordic_tb --vcd=cordic.vcd --stop-time=1000ns
```

### Matrix multiplier

```bash
cd mult_matrix
ghdl -a ../direct_kinematics/robot_config_pkg.vhd
ghdl -a ../direct_kinematics/htm_pkg.vhd
ghdl -a mult_m.vhd
ghdl -a mult_m_tb.vhd
ghdl -e mult_m_tb
ghdl -r mult_m_tb --vcd=mult_m.vcd --stop-time=1000ns
```

### Full direct kinematics pipeline

```bash
cd direct_kinematics
ghdl -a robot_config_pkg.vhd
ghdl -a ../cordic/cordic.vhd
ghdl -a htm_pkg.vhd
ghdl -a ../mult_matrix/mult_m.vhd
ghdl -a direct_kinematics.vhd
ghdl -a direct_kinematics_tb.vhd
ghdl -e direct_kinematics_tb
ghdl -r direct_kinematics_tb --vcd=ondas.vcd --stop-time=50000ns
```

Open the resulting `.vcd` file with GTKWave (or any waveform viewer of your choice) to inspect the internal state: `matrix_idx`, `t_matrix_reg`, and the per-link debug signals (`dbg_link_x/y/z`, `dbg_current_a/d`) are exposed specifically to make the accumulation process traceable step by step.

## Results

The included testbench exercises a 6-DOF configuration at two joint configurations: the home position (all angles at zero) and a 90° rotation of the base joint. The end-effector position converges correctly through all six sequential matrix multiplications, with the final translation vector matching the expected geometric result within the resolution of the Q8.24 format (roughly 6·10⁻⁸ in normalized units, i.e. sub-millimeter given the scale used for the DH parameters).

Latency for one full kinematics solve, from `start` to `done`, is fixed and fully deterministic: `ITERATIONS_CORDIC` cycles for the CORDIC stage, one cycle to assemble the DH matrices, and `DOF × (mult_m latency)` cycles for the sequential accumulation. With the default parameters (26 CORDIC iterations, `mult_m` at 4 cycles per matrix product, 6 DOF), a full solve completes in well under 60 clock cycles.

## Design notes

- **Every element of every matrix is driven in every branch.** A previous revision left one element of the DH rotation matrices unassigned across all `alpha` cases, which synthesized fine but simulated as `'U'`/`'X'` and silently propagated through the multiplier once that element reached a real operand. If you extend the `alpha_enum` or add new matrix fields, double check that all 16 elements have a driver on every path.
- **The arctangent LUT is generated, not hand-typed.** `cordic.vhd` computes its table with a VHDL function at elaboration time using `ieee.math_real`, so changing `ITERATIONS` or `FRAC_WIDTH` regenerates correct constants automatically — there's no LUT to keep in sync by hand.
- **Rounding is symmetric, not truncating.** Every fixed-point rescale in `direct_kinematics` adds half an LSB before shifting down. Removing that offset is a cheap way to save a few LUTs at the cost of a small systematic bias in the final pose.

## License

Copyright (c) 2026 Jose Segura Montes. See source file headers for licensing terms.
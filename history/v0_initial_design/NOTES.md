# v0 - initial design (superseded)

The first version of the project, kept for reference. **Do not use these files.**
They were replaced by the design in `rtl/` for the reasons below (details in the report,
"Problems Encountered and Lessons Learned").

| File | What it was | Why it was replaced |
|---|---|---|
| `rk4_seq_core.v` | One RK4 step per clock, purely combinational | Four 32x32 multipliers chained in one cycle: cannot run at 100 MHz |
| `rk4_pipe_core.v` | 4-stage pipeline, fixed to dy/dx = -y | Two multipliers chained in the last stage; 16 DSPs per core (16 cores = 256 DSPs > 220 on the XC7Z020); idle 3 cycles out of 4 because each step needs the previous result |
| `tb_rk4_parallel.v`, `tb_rk4_sequential.v` | Testbenches (16 cores / 1 core) | Run times were computed from a formula, not measured |
| `benchmark_inputs.hex` | Inputs as "x y" pairs on one line | `$readmemh` reads the two values on a line as two separate words, so half the cores started from 0 |

Measured on this version: one `rk4_pipe_core` synthesised to 316 LUTs, 317 FFs, 16 DSPs.
Also: h = 0.01 and h/6 were stored in Q16.16 (655 and 109), about 0.2 % error in h/6.

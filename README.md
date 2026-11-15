# RK4 Integration Accelerator on the Zybo Z7-20 (Zynq-7020)

ELL305 (Computer Architecture) project, IIT Delhi - Olivia (2024MT10925).

A pipelined fixed-point RK4 solver for dy/dx = a*y + b*x + c in the FPGA fabric of the Zynq-7020,
connected to the ARM Cortex-A9 as a memory-mapped AXI4-Lite peripheral. The user picks an equation
and a number of steps over UART; two timing pins let a logic analyser measure the FPGA and ARM times.

## Repository layout
| Folder | Contents |
|---|---|
| `rtl/` | Verilog: `rk4_core.v` (pipeline), `rk4_engine.v` (2 lanes, step counter), `rk4_axi.v` (AXI4-Lite top) |
| `sim/` | Self-checking testbench and expected results (`*.mem`) |
| `scripts/` | `golden_model.py` - bit-exact Python model, regenerates `sim/*.mem` |
| `vivado/` | `build.tcl` (whole project from scratch) and `rk4_la.xdc` (timing pins on Pmod JE) |
| `sw/` | `main.c` - bare-metal Vitis application (menu, polling, ARM comparison) |
| `bench/` | Original Python / C++ desktop benchmarks |
| `report/` | Project report (single LaTeX file) |
| `history/` | Superseded first version, with notes on why it was replaced |

## Tools
Vivado / Vitis 2020.1, Digilent Zybo Z7-20 board files, WaveForms 3.25 (Analog Discovery 3), Icarus Verilog for quick simulation.

## Copyright
All IPR belongs to IIT Delhi; cited material belongs to its respective authors.

---

# RK4 accelerator on Zybo Z7-20 (XC7Z020-1CLG400C)

The FPGA solves **dy/dx = a*y + b*x + c** with RK4 (h = 0.01) for **20 starting values
y0 = 10, 20, ..., 200 at once**. The ARM program shows a menu of 5 equations on the
USB serial terminal; you pick one and a number of steps, it runs the FPGA and the ARM
and prints both results, both times and the speed-up. Two timing pins let a logic
analyser measure the FPGA and ARM times independently.

Menu: 1) -y  2) -2y  3) x - y  4) 2 - 0.5y  5) x - 0.5y

```
rtl/rk4_core.v      10-stage pipelined RK4 step (1 step/clock, 10 DSPs)
rtl/rk4_engine.v    20 equations over 2 cores, step counter, b*x + c accumulator
rtl/rk4_axi.v       AXI4-Lite registers + 2 timing pins (top for "Add Module")
vivado/rk4_la.xdc   timing pins -> Pmod JE1, JE2
sim/tb_rk4_axi.v    self-checking testbench: all 5 equations bit-exact, pin checks
sim/*.mem           initial values, coefficients, expected results
scripts/golden_model.py  bit-exact Python model (regenerates sim/*.mem)
vivado/build.tcl    builds the whole project from scratch (optional)
sw/main.c           menu-driven bare-metal program (BSP stdin/stdout = ps7_uart_1)
bench/              original CPU benchmarks
```

## Setup
* Serial: Vitis Serial Terminal, COM port of the board, 115200 8N1. Type in its bottom box, press Send.
* Logic analyser (Analog Discovery 3, WaveForms -> Logic):

| AD3 | Pmod JE (upper row) | Signal |
|---|---|---|
| DIO 0 | pin 1 = rightmost hole | FPGA busy: pulse width = FPGA time |
| DIO 1 | pin 2 = 2nd from right | ARM busy: pulse width = ARM time |
| GND | pin 5 = 2nd from left | ground (pin 6, leftmost, is 3.3 V: never connect) |

Trigger: rising edge on DIO 0, single capture, about 300 ms on screen.

## Register map (base 0x43C00000)
| Offset | Name | Format |
|---|---|---|
| 0x00 | CTRL (write 1 = start) | |
| 0x04 | STATUS (bit0 done, bit1 busy) | |
| 0x08 | STEPS | |
| 0x0C | H6 = h/6 | Q1.23 |
| 0x10 | AH2 = a*h/2 | Q1.23 |
| 0x14 | AH = a*h | Q1.23 |
| 0x18 | CYCLES | |
| 0x1C | ID = 0x524B3402 | |
| 0x20 | A = a | Q4.20 |
| 0x24 | BH2 = b*h/2 | Q16.16 |
| 0x28 | BH = b*h | Q16.16 |
| 0x2C | G0 = b*x0 + c | Q16.16 |
| 0x30 | GSTEP = b*h | Q8.24 |
| 0x34 | NEQ (= 20) | |
| 0x38 | SWFLAG (bit0 -> JE2, ARM busy) | |
| 0x80.. | Y[0..19] | Q16.16 |

## Performance (simulation)
20 equations x 100 000 steps = 1 000 011 clocks = 10.0 ms at 100 MHz; FPGA-busy pin high 10.0001 ms.
Accuracy vs double precision: about 1e-4 at x = 1, about 1e-3 at x = 1000.

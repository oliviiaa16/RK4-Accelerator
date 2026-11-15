#!/usr/bin/env python3
"""
Golden model for the RK4 accelerator (dy/dx = a*y + b*x + c).

Writes, for the simulation testbench:
  sim/y_init.mem                 20 initial values y0 = 10, 20, ..., 200 (Q16.16)
  sim/coeffs_e<N>.mem            8 coefficient words for menu equation N
  sim/expected_e<N>_<S>.mem      bit-exact hardware result after S steps

and prints the fixed-point result next to a double-precision RK4 reference.

Usage:  python3 scripts/golden_model.py [steps]      (default 100)
"""
import os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N_EQ, H, X0 = 20, 0.01, 0.0

# the same menu as sw/main.c : (text, a, b, c)
MENU = {
    1: ("dy/dx = -y",        -1.0, 0.0, 0.0),
    2: ("dy/dx = -2y",       -2.0, 0.0, 0.0),
    3: ("dy/dx = x - y",     -1.0, 1.0, 0.0),
    4: ("dy/dx = 2 - 0.5y",  -0.5, 0.0, 2.0),
    5: ("dy/dx = x - 0.5y",  -0.5, 1.0, 0.0),
}


def s32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def q(v, frac):
    return int(round(v * (1 << frac)))


def rnd(p, frac):
    return s32((p + (1 << (frac - 1))) >> frac)


def coeffs(a, b, c, h=H, x0=X0):
    return dict(A=q(a, 20), AH2=q(a * h / 2, 23), AH=q(a * h, 23), H6=q(h / 6, 23),
                BH2=q(b * h / 2, 16), BH=q(b * h, 16), G0=q(b * x0 + c, 16), GSTEP=q(b * h, 24))


def hw_run(y, steps, k):
    """exactly what rk4_engine + rk4_core compute"""
    g_acc = k["G0"] << 8
    for _ in range(steps):
        g = s32((g_acc + 128) >> 8)
        k1 = s32(g + rnd(y * k["A"], 20))
        k2 = s32(k1 + rnd(k1 * k["AH2"], 23) + k["BH2"])
        k3 = s32(k1 + rnd(k2 * k["AH2"], 23) + k["BH2"])
        s8 = s32(s32(k1 + 2 * k2) + 2 * k3 + k1 + k["BH"])
        ksum = s32(s8 + rnd(k3 * k["AH"], 23))
        y = s32(y + rnd(ksum * k["H6"], 23))
        g_acc += k["GSTEP"]
    return y


def sw_rk4(y, x, h, steps, a, b, c):
    f = lambda x, y: a * y + b * x + c
    for _ in range(steps):
        k1 = f(x, y); k2 = f(x + h/2, y + h*k1/2)
        k3 = f(x + h/2, y + h*k2/2); k4 = f(x + h, y + h*k3)
        y += h/6 * (k1 + 2*k2 + 2*k3 + k4); x += h
    return y


def hexw(v):
    return f"{v & 0xFFFFFFFF:08X}\n"


def main():
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    y0 = [10.0 * (i + 1) for i in range(N_EQ)]
    yq = [q(v, 16) for v in y0]
    sim = os.path.join(ROOT, "sim")
    with open(os.path.join(sim, "y_init.mem"), "w") as fp:
        fp.writelines(hexw(v) for v in yq)

    for n, (txt, a, b, c) in MENU.items():
        k = coeffs(a, b, c)
        with open(os.path.join(sim, f"coeffs_e{n}.mem"), "w") as fp:
            fp.writelines(hexw(k[name]) for name in ("A", "AH2", "AH", "H6", "BH2", "BH", "G0", "GSTEP"))
        out, maxerr = [], 0.0
        for yv, yi in zip(y0, yq):
            r = hw_run(yi, steps, k)
            out.append(r)
            maxerr = max(maxerr, abs(r / 65536 - sw_rk4(yv, X0, H, steps, a, b, c)))
        with open(os.path.join(sim, f"expected_e{n}_{steps}.mem"), "w") as fp:
            fp.writelines(hexw(v) for v in out)
        print(f"{n}: {txt:18s} y(x={steps*H:g}) for y0=10: hw={out[0]/65536:11.6f} "
              f"double={sw_rk4(10.0, X0, H, steps, a, b, c):11.6f}   max err over 20 eq = {maxerr:.2e}")


if __name__ == "__main__":
    main()

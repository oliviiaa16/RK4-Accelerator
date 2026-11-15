/* ---------------------------------------------------------------------------
 * RK4 accelerator demo for the Zybo Z7-20 (Vitis 2020.1, standalone).
 *
 * The FPGA solves dy/dx = a*y + b*x + c for 20 starting values y0 = 10..200.
 * Over the USB serial terminal (Vitis Serial Terminal, 115200) the user
 *   1. picks an equation from a menu,
 *   2. types the number of RK4 steps.
 * The program then runs the FPGA and the ARM (double precision), checks the
 * FPGA bit-exactly against a C model of the hardware, and prints the results,
 * both run times and the speed-up.
 *
 * Serial: 115200 baud, 8 data bits, no parity, 1 stop bit.
 * BSP setting: stdin/stdout = ps7_uart_1 (the USB-UART on the PROG/UART port).
 * Logic-analyser pins: Pmod JE1 = FPGA busy, JE2 = ARM busy (software RK4);
 * their pulse widths corroborate the FPGA and ARM times printed here.
 * ------------------------------------------------------------------------- */
#include <stdint.h>
#include "xil_io.h"
#include "xil_printf.h"
#include "xtime_l.h"
#include "xparameters.h"

char inbyte(void);              /* one character from stdin (standalone BSP) */

#ifndef RK4_BASE
#define RK4_BASE 0x43C00000u    /* from Vivado's Address Editor */
#endif

#define REG_CTRL    0x00
#define REG_STATUS  0x04
#define REG_STEPS   0x08
#define REG_H6      0x0C
#define REG_AH2     0x10
#define REG_AH      0x14
#define REG_CYCLES  0x18
#define REG_ID      0x1C
#define REG_A       0x20
#define REG_BH2     0x24
#define REG_BH      0x28
#define REG_G0      0x2C
#define REG_GSTEP   0x30
#define REG_NEQ     0x34
#define REG_SWFLAG  0x38   /* drives Pmod JE2 (ARM busy) for the logic analyser */
#define REG_Y(i)    (0x80 + 4 * (i))

#define N_EQ      20
#define H         0.01
#define X0        0.0
#define PL_MHZ    100.0
#define MAX_STEPS 1000000u

/* 1 = print only 3 sample rows (y0 = 10, 100, 200) per run - easier to read in
 *     the WaveForms Protocol window, which shows the output as one stream.
 *     All 20 equations are still computed and checked bit-exactly.
 * 0 = print all 20 rows (nice in the Vitis Serial Terminal). */
#define COMPACT_OUTPUT 1

typedef struct { const char *text; double a, b, c; } equation_t;

static const equation_t MENU[] = {
    { "dy/dx = -y",        -1.0, 0.0, 0.0 },
    { "dy/dx = -2y",       -2.0, 0.0, 0.0 },
    { "dy/dx = x - y",     -1.0, 1.0, 0.0 },
    { "dy/dx = 2 - 0.5y",  -0.5, 0.0, 2.0 },
    { "dy/dx = x - 0.5y",  -0.5, 1.0, 0.0 },
};
#define N_MENU ((int)(sizeof(MENU) / sizeof(MENU[0])))

/* fixed-point coefficients sent to the hardware */
typedef struct { int32_t A, AH2, AH, H6, BH2, BH, G0, GSTEP; } coeffs_t;

static inline void     wr(uint32_t off, uint32_t v) { Xil_Out32(RK4_BASE + off, v); }
static inline uint32_t rd(uint32_t off)             { return Xil_In32(RK4_BASE + off); }

static int32_t to_q(double v, int frac)
{
    return (int32_t)(v * (double)(1u << frac) + (v >= 0 ? 0.5 : -0.5));
}

static coeffs_t make_coeffs(const equation_t *e)
{
    coeffs_t k;
    k.A     = to_q(e->a, 20);
    k.AH2   = to_q(e->a * H / 2, 23);
    k.AH    = to_q(e->a * H, 23);
    k.H6    = to_q(H / 6, 23);
    k.BH2   = to_q(e->b * H / 2, 16);
    k.BH    = to_q(e->b * H, 16);
    k.G0    = to_q(e->b * X0 + e->c, 16);
    k.GSTEP = to_q(e->b * H, 24);
    return k;
}

/* ---- printing doubles with xil_printf (no %f support) ---- */
static void print_fix(double v, int decimals)
{
    if (v < 0) { xil_printf("-"); v = -v; }
    int32_t ip = (int32_t)v;
    double frac = v - ip;
    int32_t scale = 1;
    for (int i = 0; i < decimals; i++) scale *= 10;
    int32_t fp = (int32_t)(frac * scale + 0.5);
    if (fp >= scale) { ip++; fp -= scale; }
    xil_printf("%d.", (int)ip);
    for (int32_t s = scale / 10; s > 0; s /= 10) xil_printf("%d", (int)((fp / s) % 10));
}

/* ---- software RK4 on the ARM (double precision) ---- */
static double rk4_solve(const equation_t *e, double y0, double x0, double h, int steps)
{
    const double a = e->a, b = e->b, c = e->c;
    double y = y0, x = x0;
    for (int i = 0; i < steps; ++i) {
        double k1 = a * y + b * x + c;
        double k2 = a * (y + h * k1 / 2.0) + b * (x + h / 2.0) + c;
        double k3 = a * (y + h * k2 / 2.0) + b * (x + h / 2.0) + c;
        double k4 = a * (y + h * k3) + b * (x + h) + c;
        y += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        x += h;
    }
    return y;
}

/* ---- bit-exact C model of the FPGA (rk4_engine + rk4_core) ---- */
static int32_t rnd(int64_t p, int frac) { return (int32_t)((p + (1LL << (frac - 1))) >> frac); }

static int32_t hw_model(int32_t y, int steps, const coeffs_t *k)
{
    int64_t g_acc = (int64_t)k->G0 * 256;
    for (int i = 0; i < steps; i++) {
        int32_t g    = (int32_t)((g_acc + 128) >> 8);
        int32_t k1   = g + rnd((int64_t)y * k->A, 20);
        int32_t k2   = k1 + rnd((int64_t)k1 * k->AH2, 23) + k->BH2;
        int32_t k3   = k1 + rnd((int64_t)k2 * k->AH2, 23) + k->BH2;
        int32_t s8   = k1 + 2 * k2 + 2 * k3 + k1 + k->BH;
        int32_t ksum = s8 + rnd((int64_t)k3 * k->AH, 23);
        y = y + rnd((int64_t)ksum * k->H6, 23);
        g_acc += k->GSTEP;
    }
    return y;
}

/* ---- one FPGA run; returns microseconds from start to done ---- */
static double run_hw(const coeffs_t *k, const int32_t *y_in, int32_t *y_out,
                     uint32_t steps, uint32_t *cycles)
{
    wr(REG_A, k->A);     wr(REG_AH2, k->AH2); wr(REG_AH, k->AH);   wr(REG_H6, k->H6);
    wr(REG_BH2, k->BH2); wr(REG_BH, k->BH);   wr(REG_G0, k->G0);   wr(REG_GSTEP, k->GSTEP);
    wr(REG_STEPS, steps);
    for (int i = 0; i < N_EQ; i++) wr(REG_Y(i), (uint32_t)y_in[i]);

    XTime t0, t1;
    XTime_GetTime(&t0);
    wr(REG_CTRL, 1);
    while (!(rd(REG_STATUS) & 1)) ;
    XTime_GetTime(&t1);

    for (int i = 0; i < N_EQ; i++) y_out[i] = (int32_t)rd(REG_Y(i));
    *cycles = rd(REG_CYCLES);
    return (double)(t1 - t0) * 1e6 / COUNTS_PER_SECOND;
}

/* ---- same work on the ARM; returns microseconds ---- */
static double run_sw(const equation_t *e, const double *y0, int steps, double *y_out)
{
    XTime t0, t1;
    wr(REG_SWFLAG, 1);                       /* JE2 high: ARM busy */
    XTime_GetTime(&t0);
    for (int i = 0; i < N_EQ; i++) y_out[i] = rk4_solve(e, y0[i], X0, H, steps);
    XTime_GetTime(&t1);
    wr(REG_SWFLAG, 0);
    return (double)(t1 - t0) * 1e6 / COUNTS_PER_SECOND;
}

/* ---- read a positive number; Enter / line feed finishes it ---- */
static uint32_t read_number(void)
{
    uint32_t v = 0;
    int digits = 0;
    for (;;) {
        char c = inbyte();
        if (c >= '0' && c <= '9') {
            if (v < 100000000u) v = v * 10 + (uint32_t)(c - '0');
            digits++;
            xil_printf("%c", c);
        } else if ((c == '\r' || c == '\n') && digits > 0) {
            xil_printf("\r\n");
            return v;
        }
    }
}

int main(void)
{
    double  y0[N_EQ], ysw[N_EQ];
    int32_t yq[N_EQ], yhw[N_EQ];
    uint32_t cycles;

    for (int i = 0; i < N_EQ; i++) { y0[i] = 10.0 * (i + 1); yq[i] = to_q(y0[i], 16); }

    xil_printf("\r\n===== RK4 accelerator on Zybo Z7-20 =====\r\n");
    uint32_t id = rd(REG_ID);
    xil_printf("ID register: 0x%08x %s\r\n", id,
               id == 0x524B3402u ? "(ok)" : "(WRONG - new bitstream loaded? base address right?)");
    if (id != 0x524B3402u) return 1;
    xil_printf("%d equations solved in parallel, y0 = 10, 20, ..., 200, x0 = 0, h = 0.01\r\n",
               (int)rd(REG_NEQ));

    for (;;) {
        xil_printf("\r\nChoose an equation:\r\n");
        for (int i = 0; i < N_MENU; i++) xil_printf("  %d : %s\r\n", i + 1, MENU[i].text);
        xil_printf("Enter 1-%d and press Enter/Send:\r\n> ", N_MENU);
        uint32_t sel = read_number();
        if (sel < 1 || sel > (uint32_t)N_MENU) { xil_printf("Invalid choice.\r\n"); continue; }
        const equation_t *e = &MENU[sel - 1];

        xil_printf("Enter number of RK4 steps (1 - %d), e.g. 100 or 100000:\r\n> ", (int)MAX_STEPS);
        uint32_t steps = read_number();
        if (steps < 1 || steps > MAX_STEPS) { xil_printf("Out of range.\r\n"); continue; }

        coeffs_t k = make_coeffs(e);
        double t_hw = run_hw(&k, yq, yhw, steps, &cycles);
        double t_sw = run_sw(e, y0, (int)steps, ysw);

        xil_printf("\r\n--- %s, %d steps (x: 0 -> ", e->text, (int)steps);
        print_fix(steps * H, 2);
        xil_printf(") ---\r\n");
        xil_printf("   y0        FPGA           ARM double     |err|     bit-exact\r\n");
        int bad = 0;
        double maxerr = 0;
        for (int i = 0; i < N_EQ; i++) {
            double hw  = yhw[i] / 65536.0;
            double err = hw > ysw[i] ? hw - ysw[i] : ysw[i] - hw;
            int exact  = (yhw[i] == hw_model(yq[i], (int)steps, &k));
            if (!exact) bad++;
            if (err > maxerr) maxerr = err;
            if (COMPACT_OUTPUT && !(i == 0 || i == 9 || i == N_EQ - 1)) continue;
            xil_printf("  "); print_fix(y0[i], 1);
            xil_printf("   ");  print_fix(hw, 6);
            xil_printf("   ");  print_fix(ysw[i], 6);
            xil_printf("   ");  print_fix(err, 6);
            xil_printf("   %s\r\n", exact ? "yes" : "NO");
        }
        if (COMPACT_OUTPUT) xil_printf("  (3 of %d rows shown; all %d checked)\r\n", N_EQ, N_EQ);
        xil_printf("max |FPGA - double| = "); print_fix(maxerr, 6);
        xil_printf("   %s\r\n", bad ? "*** BIT-EXACT CHECK FAILED ***" : "bit-exact with hardware model: PASS");
        xil_printf("FPGA : %d cycles = ", (int)cycles); print_fix(cycles / PL_MHZ / 1000.0, 3);
        xil_printf(" ms (ARM timer: "); print_fix(t_hw / 1000.0, 3); xil_printf(" ms)\r\n");
        xil_printf("ARM  : "); print_fix(t_sw / 1000.0, 3); xil_printf(" ms (Cortex-A9, double)\r\n");
        xil_printf("Speed-up: "); print_fix(t_sw / t_hw, 2); xil_printf(" x\r\n");
    }
    return 0;
}

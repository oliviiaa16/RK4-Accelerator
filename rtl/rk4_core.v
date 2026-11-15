`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// rk4_core : fully pipelined RK4 step for the linear ODE family
//
//          dy/dx = f(x, y) = a*y + b*x + c
//
// With g = b*x + c (supplied per step by the engine) the RK4 stages become
//   k1 = a*y + g
//   k2 = k1 + (a*h/2)*k1 + b*h/2
//   k3 = k1 + (a*h/2)*k2 + b*h/2
//   k4 = k1 + (a*h)  *k3 + b*h
//   y' = y + (h/6)*(k1 + 2k2 + 2k3 + k4)
//
// Formats
//   y, g, BH2, BH : Q16.16 signed 32 bit
//   A             : Q4.20  signed 25 bit  (|a| < 16)
//   AH2, AH, H6   : Q1.23  signed 25 bit  (a*h/2, a*h, h/6)
//
// One new y enters every clock; the result appears LAT = 10 clocks later.
// Every multiply has its own register stage (each is 32x25 -> 2 DSP48E1),
// 5 multiplies -> 10 DSPs per core.
// ---------------------------------------------------------------------------
module rk4_core #(
    parameter FRAC_H = 23,
    parameter FRAC_A = 20,
    parameter TAGW   = 4
)(
    input  wire                  clk,
    input  wire                  rst,
    input  wire                  in_valid,
    input  wire [TAGW-1:0]       in_tag,
    input  wire signed [31:0]    y_in,
    input  wire signed [31:0]    g_in,
    input  wire signed [24:0]    A,
    input  wire signed [24:0]    AH2,
    input  wire signed [24:0]    AH,
    input  wire signed [24:0]    H6,
    input  wire signed [31:0]    BH2,
    input  wire signed [31:0]    BH,
    output wire                  out_valid,
    output wire [TAGW-1:0]       out_tag,
    output wire signed [31:0]    y_next,      // combinational from last register stage
    output wire                  active
);
    localparam LAT = 10;
    localparam signed [56:0] RND_H = 57'sd1 <<< (FRAC_H-1);
    localparam signed [56:0] RND_A = 57'sd1 <<< (FRAC_A-1);

    reg [LAT-1:0]  v;
    reg [TAGW-1:0] t [0:LAT-1];
    integer i;
    always @(posedge clk) begin
        if (rst) v <= 0;
        else     v <= {v[LAT-2:0], in_valid};
        t[0] <= in_tag;
        for (i = 1; i < LAT; i = i + 1) t[i] <= t[i-1];
    end

    // S1 : input register
    reg signed [31:0] y1, g1;
    always @(posedge clk) begin y1 <= y_in; g1 <= g_in; end

    // S2 : pA = a*y
    reg signed [56:0] pA;  reg signed [31:0] y2, g2;
    always @(posedge clk) begin pA <= y1 * A; y2 <= y1; g2 <= g1; end

    // S3 : k1 = a*y + g
    wire signed [56:0] rA = pA + RND_A;
    wire signed [31:0] mA = rA >>> FRAC_A;
    reg  signed [31:0] k1_3, y3;
    always @(posedge clk) begin k1_3 <= g2 + mA; y3 <= y2; end

    // S4 : p2 = (a*h/2)*k1
    reg signed [56:0] p2;  reg signed [31:0] y4, k1_4;
    always @(posedge clk) begin p2 <= k1_3 * AH2; y4 <= y3; k1_4 <= k1_3; end

    // S5 : k2 = k1 + p2 + b*h/2
    wire signed [56:0] r2 = p2 + RND_H;
    wire signed [31:0] m2 = r2 >>> FRAC_H;
    reg  signed [31:0] k2_5, y5, k1_5;
    always @(posedge clk) begin k2_5 <= k1_4 + m2 + BH2; y5 <= y4; k1_5 <= k1_4; end

    // S6 : p3 = (a*h/2)*k2 ; s = k1 + 2*k2
    reg signed [56:0] p3;  reg signed [31:0] y6, k1_6, s6;
    always @(posedge clk) begin
        p3 <= k2_5 * AH2; y6 <= y5; k1_6 <= k1_5; s6 <= k1_5 + (k2_5 <<< 1);
    end

    // S7 : k3 = k1 + p3 + b*h/2
    wire signed [56:0] r3 = p3 + RND_H;
    wire signed [31:0] m3 = r3 >>> FRAC_H;
    reg  signed [31:0] k3_7, y7, k1_7, s7;
    always @(posedge clk) begin k3_7 <= k1_6 + m3 + BH2; y7 <= y6; k1_7 <= k1_6; s7 <= s6; end

    // S8 : p4 = (a*h)*k3 ; s = k1 + 2k2 + 2k3 + (k1 + b*h)   (k4 = k1 + p4 + b*h)
    reg signed [56:0] p4;  reg signed [31:0] y8, s8;
    always @(posedge clk) begin
        p4 <= k3_7 * AH; y8 <= y7; s8 <= s7 + (k3_7 <<< 1) + k1_7 + BH;
    end

    // S9 : ksum = k1 + 2k2 + 2k3 + k4
    wire signed [56:0] r4 = p4 + RND_H;
    wire signed [31:0] m4 = r4 >>> FRAC_H;
    reg  signed [31:0] ksum9, y9;
    always @(posedge clk) begin ksum9 <= s8 + m4; y9 <= y8; end

    // S10 : pf = (h/6)*ksum
    reg signed [56:0] pf;  reg signed [31:0] y10;
    always @(posedge clk) begin pf <= ksum9 * H6; y10 <= y9; end

    // output : y' = y + pf
    wire signed [56:0] rf = pf + RND_H;
    wire signed [31:0] mf = rf >>> FRAC_H;
    assign y_next    = y10 + mf;
    assign out_valid = v[LAT-1];
    assign out_tag   = t[LAT-1];
    assign active    = |v;
endmodule

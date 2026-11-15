`timescale 1ns / 1ps
module rk4_seq_core (
    input wire signed [31:0] y_in,
    input wire signed [31:0] h,
    input wire signed [31:0] h_over_6,
    output wire signed [31:0] y_out
);
    wire signed [31:0] k1, k2, k3, k4;
    assign k1 = -y_in;
    wire signed [31:0] h_half = h >>> 1;
    wire signed [63:0] mult_k2 = (h_half * k1) >>> 16;
    assign k2 = -(y_in + mult_k2[31:0]);
    wire signed [63:0] mult_k3 = (h_half * k2) >>> 16;
    assign k3 = -(y_in + mult_k3[31:0]);
    wire signed [63:0] mult_k4 = (h * k3) >>> 16;
    assign k4 = -(y_in + mult_k4[31:0]);
    wire signed [31:0] k_sum = k1 + (k2 <<< 1) + (k3 <<< 1) + k4;
    wire signed [63:0] final_mult = (h_over_6 * k_sum) >>> 16;
    assign y_out = y_in + final_mult[31:0];
endmodule

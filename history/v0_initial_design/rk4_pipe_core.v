`timescale 1ns / 1ps
module rk4_pipe_core (
    input wire clk,
    input wire reset,
    input wire signed [31:0] y_in,
    input wire signed [31:0] h,
    input wire signed [31:0] h_over_6,
    output reg signed [31:0] y_out
);
    wire signed [31:0] k1_comb = -y_in;
    reg signed [31:0] k1_s1, y_in_s1;
    always @(posedge clk) begin
        if (reset) begin k1_s1 <= 0; y_in_s1 <= 0; end
        else begin k1_s1 <= k1_comb; y_in_s1 <= y_in; end
    end

    wire signed [31:0] h_half = h >>> 1;
    wire signed [63:0] mult_k2 = (h_half * k1_s1) >>> 16;
    wire signed [31:0] k2_comb = -(y_in_s1 + mult_k2[31:0]);
    reg signed [31:0] k1_s2, k2_s2, y_in_s2;
    always @(posedge clk) begin
        if (reset) begin k1_s2 <= 0; k2_s2 <= 0; y_in_s2 <= 0; end
        else begin k1_s2 <= k1_s1; k2_s2 <= k2_comb; y_in_s2 <= y_in_s1; end
    end

    wire signed [63:0] mult_k3 = (h_half * k2_s2) >>> 16;
    wire signed [31:0] k3_comb = -(y_in_s2 + mult_k3[31:0]);
    reg signed [31:0] k1_s3, k2_s3, k3_s3, y_in_s3;
    always @(posedge clk) begin
        if (reset) begin k1_s3 <= 0; k2_s3 <= 0; k3_s3 <= 0; y_in_s3 <= 0; end
        else begin k1_s3 <= k1_s2; k2_s3 <= k2_s2; k3_s3 <= k3_comb; y_in_s3 <= y_in_s2; end
    end

    wire signed [63:0] mult_k4 = (h * k3_s3) >>> 16;
    wire signed [31:0] k4_comb = -(y_in_s3 + mult_k4[31:0]);
    wire signed [31:0] k_sum = k1_s3 + (k2_s3 <<< 1) + (k3_s3 <<< 1) + k4_comb;
    wire signed [63:0] final_mult = (h_over_6 * k_sum) >>> 16;
    wire signed [31:0] y_out_comb = y_in_s3 + final_mult[31:0];
    always @(posedge clk) begin
        if (reset) y_out <= 0;
        else y_out <= y_out_comb;
    end
endmodule

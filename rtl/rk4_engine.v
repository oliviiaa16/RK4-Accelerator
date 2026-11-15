`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// rk4_engine : runs N_EQ independent RK4 integrations of
//              dy/dx = a*y + b*x + c   for STEPS steps.
//
// The equations are split over NLANES pipelined cores; each core handles
// SLOTS = N_EQ/NLANES equations round-robin, so with SLOTS >= LAT (10) every
// core finishes one RK4 step per clock:
//
//   total clocks ~= STEPS * SLOTS   (20 eq, 2 lanes -> 10 clocks per step)
//
// g = b*x + c is the same for all equations at a given step. It is kept in a
// high-precision accumulator (24 fractional bits) and advanced by b*h once per
// sweep, so x does not drift over millions of steps.
// ---------------------------------------------------------------------------
module rk4_engine #(
    parameter N_EQ   = 20,
    parameter NLANES = 2
)(
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire [31:0]        steps,
    input  wire signed [24:0] A, AH2, AH, H6,
    input  wire signed [31:0] BH2, BH,
    input  wire signed [31:0] G0,        // b*x0 + c        (Q16.16)
    input  wire signed [31:0] GSTEP,     // b*h             (Q8.24)
    output reg                busy,
    output reg                done,
    output reg  [31:0]        cycles,
    input  wire               y_we,
    input  wire [$clog2(N_EQ)-1:0] y_waddr,
    input  wire [31:0]        y_wdata,
    input  wire [$clog2(N_EQ)-1:0] y_raddr,
    output wire [31:0]        y_rdata
);
    localparam SLOTS = N_EQ / NLANES;
    localparam TAGW  = (SLOTS > 1) ? $clog2(SLOTS) : 1;
    localparam LAT   = 10;

    generate if (SLOTS < LAT || (N_EQ % NLANES) != 0) begin : bad_cfg
        initial begin
            $display("rk4_engine: need N_EQ/NLANES >= %0d and N_EQ divisible by NLANES", LAT);
            $finish;
        end
    end endgenerate

    reg signed [31:0] y_mem [0:N_EQ-1];
    assign y_rdata = (y_raddr < N_EQ) ? y_mem[y_raddr] : 32'd0;

    reg [TAGW-1:0] slot;
    reg [31:0]     sweep;
    wire issuing = busy && (sweep < steps);

    // g = b*x + c accumulator, Q24.24
    reg  signed [47:0] g_acc;
    wire signed [47:0] g_rnd   = g_acc + 48'sd128;
    wire signed [31:0] g_issue = g_rnd >>> 8;

    wire [NLANES-1:0]  o_valid, lane_active;
    wire [TAGW-1:0]    o_tag [0:NLANES-1];
    wire signed [31:0] o_y   [0:NLANES-1];
    wire signed [31:0] i_y   [0:NLANES-1];

    genvar l;
    generate for (l = 0; l < NLANES; l = l + 1) begin : lane
        assign i_y[l] = (o_valid[l] && o_tag[l] == slot) ? o_y[l] : y_mem[l*SLOTS + slot];

        rk4_core #(.TAGW(TAGW)) core (
            .clk(clk), .rst(rst),
            .in_valid(issuing), .in_tag(slot), .y_in(i_y[l]), .g_in(g_issue),
            .A(A), .AH2(AH2), .AH(AH), .H6(H6), .BH2(BH2), .BH(BH),
            .out_valid(o_valid[l]), .out_tag(o_tag[l]), .y_next(o_y[l]),
            .active(lane_active[l])
        );
    end endgenerate

    integer k;
    always @(posedge clk) begin
        if (!busy && y_we && y_waddr < N_EQ) y_mem[y_waddr] <= y_wdata;
        for (k = 0; k < NLANES; k = k + 1)
            if (o_valid[k]) y_mem[k*SLOTS + o_tag[k]] <= o_y[k];
    end

    always @(posedge clk) begin
        if (rst) begin
            busy <= 0; done <= 0; slot <= 0; sweep <= 0; cycles <= 0; g_acc <= 0;
        end else if (!busy) begin
            if (start) begin
                busy <= 1; done <= 0; slot <= 0; sweep <= 0; cycles <= 0;
                g_acc <= $signed(G0) <<< 8;
            end
        end else begin
            cycles <= cycles + 1;
            if (issuing) begin
                if (slot == SLOTS-1) begin
                    slot  <= 0;
                    sweep <= sweep + 1;
                    g_acc <= g_acc + $signed(GSTEP);     // x -> x + h
                end else
                    slot <= slot + 1;
            end else if (!(|lane_active)) begin
                busy <= 0; done <= 1;
            end
        end
    end
endmodule

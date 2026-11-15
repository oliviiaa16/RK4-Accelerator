`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// rk4_axi : AXI4-Lite slave wrapper around rk4_engine (Verilog-2001; add it to
//           the block design with "Add Module").
//
// Solves dy/dx = a*y + b*x + c for N_EQ (=20) starting values at once.
//
// Register map (byte offsets, 32-bit):
//   0x00 CTRL   W : bit0 = start
//   0x04 STATUS R : bit0 = done, bit1 = busy
//   0x08 STEPS  RW: number of RK4 steps
//   0x0C H6     RW: h/6        Q1.23
//   0x10 AH2    RW: a*h/2      Q1.23
//   0x14 AH     RW: a*h        Q1.23
//   0x18 CYCLES R : clocks used by the last run
//   0x1C ID     R : 0x524B3402 ("RK4", v2)
//   0x20 A      RW: a          Q4.20
//   0x24 BH2    RW: b*h/2      Q16.16
//   0x28 BH     RW: b*h        Q16.16
//   0x2C G0     RW: b*x0 + c   Q16.16
//   0x30 GSTEP  RW: b*h        Q8.24
//   0x34 NEQ    R : number of equations (20)
//   0x38 SWFLAG RW: bit0 -> la_out[1] (ARM sets it while running software RK4)
//
// Logic-analyser outputs (pinned to Pmod JE by rk4_la.xdc):
//   la_out[0] = FPGA busy (high while the accelerator runs)   -> JE1
//   la_out[1] = ARM busy  (SWFLAG, high during software RK4)  -> JE2
//   0x80..      Y[i] RW: Q16.16 values (write y0 before start, read result after done)
// ---------------------------------------------------------------------------
module rk4_axi #(
    parameter N_EQ   = 20,
    parameter NLANES = 2
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
    input  wire        s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        s_axi_aresetn,

    input  wire [7:0]  s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    input  wire [7:0]  s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    output wire [1:0]  la_out
);
    localparam AW_Y = $clog2(N_EQ);
    wire clk = s_axi_aclk;
    wire rst = ~s_axi_aresetn;

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    reg [31:0] r_steps, r_h6, r_ah2, r_ah, r_a, r_bh2, r_bh, r_g0, r_gstep;
    reg        r_swflag;
    reg        start_pulse;
    wire       busy, done;
    wire [31:0] cycles, y_rdata;

    // ---------------- write channel ----------------
    wire wr_en = s_axi_awvalid && s_axi_wvalid && !s_axi_awready && !s_axi_bvalid;
    wire [7:0] wa = s_axi_awaddr;
    wire y_we  = wr_en && wa[7];

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready <= 0; s_axi_wready <= 0; s_axi_bvalid <= 0;
            start_pulse <= 0;
            // defaults: dy/dx = -y, h = 0.01
            r_steps <= 32'd100;
            r_h6  <= 32'd13981;              // 0.01/6        Q1.23
            r_ah2 <= -32'sd41943;            // -0.005        Q1.23
            r_ah  <= -32'sd83886;            // -0.01         Q1.23
            r_a   <= -32'sd1048576;          // -1            Q4.20
            r_bh2 <= 0; r_bh <= 0; r_g0 <= 0; r_gstep <= 0; r_swflag <= 0;
        end else begin
            s_axi_awready <= wr_en;
            s_axi_wready  <= wr_en;
            start_pulse   <= 0;
            if (s_axi_awready)                     s_axi_bvalid <= 1;
            else if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 0;

            if (wr_en && !wa[7]) begin
                case (wa[6:2])
                    5'h00: start_pulse <= s_axi_wdata[0];
                    5'h02: r_steps <= s_axi_wdata;
                    5'h03: r_h6    <= s_axi_wdata;
                    5'h04: r_ah2   <= s_axi_wdata;
                    5'h05: r_ah    <= s_axi_wdata;
                    5'h08: r_a     <= s_axi_wdata;
                    5'h09: r_bh2   <= s_axi_wdata;
                    5'h0A: r_bh    <= s_axi_wdata;
                    5'h0B: r_g0    <= s_axi_wdata;
                    5'h0C: r_gstep <= s_axi_wdata;
                    5'h0E: r_swflag <= s_axi_wdata[0];
                    default: ;
                endcase
            end
        end
    end

    // ---------------- read channel ----------------
    reg  [7:0] ra;
    wire rd_en = s_axi_arvalid && !s_axi_arready && !s_axi_rvalid;
    always @(posedge clk) begin
        if (rst) begin
            s_axi_arready <= 0; s_axi_rvalid <= 0; s_axi_rdata <= 0; ra <= 0;
        end else begin
            s_axi_arready <= rd_en;
            if (rd_en) ra <= s_axi_araddr;
            if (s_axi_arready) begin
                s_axi_rvalid <= 1;
                if (ra[7]) s_axi_rdata <= y_rdata;
                else case (ra[6:2])
                    5'h01: s_axi_rdata <= {30'd0, busy, done};
                    5'h02: s_axi_rdata <= r_steps;
                    5'h03: s_axi_rdata <= r_h6;
                    5'h04: s_axi_rdata <= r_ah2;
                    5'h05: s_axi_rdata <= r_ah;
                    5'h06: s_axi_rdata <= cycles;
                    5'h07: s_axi_rdata <= 32'h524B3402;
                    5'h08: s_axi_rdata <= r_a;
                    5'h09: s_axi_rdata <= r_bh2;
                    5'h0A: s_axi_rdata <= r_bh;
                    5'h0B: s_axi_rdata <= r_g0;
                    5'h0C: s_axi_rdata <= r_gstep;
                    5'h0D: s_axi_rdata <= N_EQ;
                    5'h0E: s_axi_rdata <= {31'd0, r_swflag};
                    default: s_axi_rdata <= 32'd0;
                endcase
            end else if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 0;
        end
    end

    // ---------------- logic-analyser pins ----------------
    reg [1:0] la_r;
    always @(posedge clk) la_r <= {r_swflag, busy};
    assign la_out = la_r;

    // ---------------- engine ----------------
    rk4_engine #(.N_EQ(N_EQ), .NLANES(NLANES)) engine (
        .clk(clk), .rst(rst),
        .start(start_pulse), .steps(r_steps),
        .A(r_a[24:0]), .AH2(r_ah2[24:0]), .AH(r_ah[24:0]), .H6(r_h6[24:0]),
        .BH2(r_bh2), .BH(r_bh), .G0(r_g0), .GSTEP(r_gstep),
        .busy(busy), .done(done), .cycles(cycles),
        .y_we(y_we), .y_waddr(wa[AW_Y+1:2]), .y_wdata(s_axi_wdata),
        .y_raddr(ra[AW_Y+1:2]), .y_rdata(y_rdata)
    );
endmodule

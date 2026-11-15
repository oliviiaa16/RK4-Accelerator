`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// Self-checking testbench for rk4_axi (dy/dx = a*y + b*x + c, 20 equations).
// Drives AXI4-Lite like the ARM, checks every menu equation bit-exactly
// against the golden model (scripts/golden_model.py), then runs the
// 100000-step benchmark and reports the cycle count.
// Needs y_init.mem, coeffs_e*.mem, expected_e*_100.mem, expected_e3_1000.mem
// ---------------------------------------------------------------------------
module tb_rk4_axi;
    localparam N = 20;
    reg clk = 0, rstn = 0;
    always #5 clk = ~clk;

    reg  [7:0]  awaddr = 0, araddr = 0;
    reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg  [31:0] wdata = 0;
    wire        awready, wready, bvalid, arready, rvalid;
    wire [31:0] rdata;
    wire [1:0]  bresp, rresp;
    wire [1:0]  la;
    realtime busy_rise = 0, busy_width = 0;
    always @(posedge la[0]) busy_rise = $realtime;
    always @(negedge la[0]) busy_width = $realtime - busy_rise;

    rk4_axi dut (
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'b0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'b0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .la_out(la)
    );

    task axi_write(input [7:0] a, input [31:0] d);
        begin
            @(posedge clk); awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1; bready <= 1;
            @(posedge clk); while (!(awready && wready)) @(posedge clk);
            awvalid <= 0; wvalid <= 0;
            while (!bvalid) @(posedge clk);
            @(posedge clk); bready <= 0;
        end
    endtask

    task axi_read(input [7:0] a, output [31:0] d);
        begin
            @(posedge clk); araddr <= a; arvalid <= 1; rready <= 1;
            @(posedge clk); while (!arready) @(posedge clk);
            arvalid <= 0;
            while (!rvalid) @(posedge clk);
            d = rdata;
            @(posedge clk); rready <= 0;
        end
    endtask

    reg [31:0] y_init [0:N-1];
    reg [31:0] expect [0:N-1];
    reg [31:0] co [0:7];          // A, AH2, AH, H6, BH2, BH, G0, GSTEP
    reg [31:0] d, cyc;
    integer i, e, errors, bad;

    task load_eq(input integer n);
        begin
            case (n)
                1: $readmemh("coeffs_e1.mem", co);
                2: $readmemh("coeffs_e2.mem", co);
                3: $readmemh("coeffs_e3.mem", co);
                4: $readmemh("coeffs_e4.mem", co);
                5: $readmemh("coeffs_e5.mem", co);
            endcase
            axi_write(8'h20, co[0]); axi_write(8'h10, co[1]); axi_write(8'h14, co[2]);
            axi_write(8'h0C, co[3]); axi_write(8'h24, co[4]); axi_write(8'h28, co[5]);
            axi_write(8'h2C, co[6]); axi_write(8'h30, co[7]);
        end
    endtask

    task run(input [31:0] steps);
        begin
            axi_write(8'h08, steps);
            for (i = 0; i < N; i = i + 1) axi_write(8'h80 + 4*i, y_init[i]);
            axi_write(8'h00, 32'd1);
            d = 0;
            while (!d[0]) axi_read(8'h04, d);
            axi_read(8'h18, cyc);
        end
    endtask

    task check(input [8*24-1:0] name);
        begin
            bad = 0;
            for (i = 0; i < N; i = i + 1) begin
                axi_read(8'h80 + 4*i, d);
                if (d !== expect[i]) bad = bad + 1;
            end
            axi_read(8'h80, d);
            $display("%0s  y(y0=10) = %10.6f  cycles = %0d  %0s", name, $signed(d) / 65536.0, cyc,
                     bad ? "MISMATCH" : "all 20 bit-exact");
            errors = errors + bad;
        end
    endtask

    initial begin
        $readmemh("y_init.mem", y_init);
        errors = 0;
        repeat (5) @(posedge clk); rstn = 1;

        axi_read(8'h1C, d);
        $display("ID = %08h %0s", d, (d == 32'h524B3402) ? "(ok)" : "(WRONG)");
        if (d != 32'h524B3402) errors = errors + 1;
        axi_read(8'h34, d);
        $display("NEQ = %0d", d);

        for (e = 1; e <= 5; e = e + 1) begin
            load_eq(e);
            run(100);
            case (e)
                1: begin $readmemh("expected_e1_100.mem", expect); check("eq1 -y        100 steps"); end
                2: begin $readmemh("expected_e2_100.mem", expect); check("eq2 -2y       100 steps"); end
                3: begin $readmemh("expected_e3_100.mem", expect); check("eq3 x-y       100 steps"); end
                4: begin $readmemh("expected_e4_100.mem", expect); check("eq4 2-0.5y    100 steps"); end
                5: begin $readmemh("expected_e5_100.mem", expect); check("eq5 x-0.5y    100 steps"); end
            endcase
        end
        load_eq(5); run(1000);
        $readmemh("expected_e5_1000.mem", expect); check("eq5 x-0.5y   1000 steps");

        load_eq(1); run(100000);
        $display("benchmark 20 eq x 100000 steps: %0d cycles -> %0.3f ms @ 100 MHz; busy pin high %0.4f ms",
                 cyc, cyc / 100000.0, busy_width / 1.0e6);
        axi_write(8'h38, 1); repeat (3) @(posedge clk);
        if (la[1] !== 1'b1) begin $display("ARM-busy pin did not go high"); errors = errors + 1; end
        axi_write(8'h38, 0); repeat (3) @(posedge clk);
        if (la[1] !== 1'b0) begin $display("ARM-busy pin did not go low"); errors = errors + 1; end

        if (errors == 0) $display("*** TEST PASSED ***");
        else             $display("*** TEST FAILED: %0d errors ***", errors);
        $finish;
    end
endmodule

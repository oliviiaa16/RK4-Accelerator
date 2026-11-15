`timescale 1ns / 1ps
module tb_rk4_sequential;
    reg clk;
    reg reset;
    reg signed [31:0] y_reg;
    reg signed [31:0] h = 32'd655;
    reg signed [31:0] h_over_6 = 32'd109;
    wire signed [31:0] y_next;
    
    rk4_seq_core uut (.y_in(y_reg), .h(h), .h_over_6(h_over_6), .y_out(y_next));

    reg [63:0] coord_mem [0:15]; 
    integer i, step_count;
    always #5 clk = ~clk;

    initial begin
        $readmemh("benchmark_inputs.hex", coord_mem);
        clk = 0; reset = 1; #20 reset = 0;
        
        $display("--- Verilog Sequential Benchmark (1 Core) ---");
        for (i = 0; i < 16; i = i + 1) begin
            y_reg = coord_mem[i][31:0]; 
            step_count = 0;
            while (step_count < 100000) begin
                @(posedge clk);
                y_reg = y_next;
                step_count = step_count + 1;
            end
        end
        $display("Processed 16 equations sequentially.");
        $display("Total Runtime @ 100MHz: %f ms", (16 * 100000 * 10.0) / 1000000.0);
        $finish;
    end
endmodule

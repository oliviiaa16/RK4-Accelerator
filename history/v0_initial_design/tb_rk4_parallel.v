`timescale 1ns / 1ps
module tb_rk4_parallel;
    reg clk;
    reg reset;
    reg signed [31:0] h = 32'd655;
    reg signed [31:0] h_over_6 = 32'd109;
    
    reg signed [31:0] y_in [0:15];
    wire signed [31:0] y_out [0:15];
    
    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : cores
            rk4_pipe_core core (
                .clk(clk), .reset(reset), .y_in(y_in[g]),
                .h(h), .h_over_6(h_over_6), .y_out(y_out[g])
            );
        end
    endgenerate
    
    reg [63:0] coord_mem [0:15]; 
    integer step_count, i;
    always #5 clk = ~clk;

    initial begin
        $readmemh("benchmark_inputs.hex", coord_mem);
        clk = 0; reset = 1; 
        
        for (i = 0; i < 16; i = i + 1) begin
            y_in[i] = coord_mem[i][31:0];
        end
        #20 reset = 0;
        
        $display("--- Verilog Parallel Benchmark (16 Cores) ---");
        step_count = 0;
        
        while (step_count < 100000) begin
            repeat(4) @(posedge clk);
            for (i = 0; i < 16; i = i + 1) begin
                y_in[i] = y_out[i];
            end
            step_count = step_count + 1;
        end
        
        $display("Processed 16 equations concurrently.");
        $display("Total Runtime @ 100MHz: %f ms", (100000 * 40.0) / 1000000.0);
        $finish;
    end
endmodule

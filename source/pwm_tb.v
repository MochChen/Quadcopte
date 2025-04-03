`timescale 1ns/1ps
`include "pwm.v"
module pwm_tb;
    reg clk;
    reg rst_n;
    reg [15:0] speed_in;
    reg speed_oe;
    wire pwm_out;
    wire busy;

    // Instantiate the bb_pwm module
    pwm uut (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(speed_in),
        .speed_oe(speed_oe),
        .pwm_out(pwm_out),
        .busy(busy)
    );

    // Generate clock
    always #5 clk = ~clk; // 10ns clock period (100 MHz)

    initial begin
        // Initialize signals
        clk = 0;
        rst_n = 1;
        speed_in = 16'd256;
        speed_oe = 0;
        
        // Reset sequence
        #20 rst_n = 0;
        #10 rst_n = 1;
        
        // Apply test cases
        #50 speed_in = 16'd56000; speed_oe = 1; // Set speed to 5000
        #20 speed_oe = 0;

        #500 $finish; // End simulation
    end

    // Monitor signals
    initial begin
        $monitor("Time=%0t | speed_in=%d | pwm_out=%b | busy=%b", $time, speed_in, pwm_out, busy);
    end

endmodule

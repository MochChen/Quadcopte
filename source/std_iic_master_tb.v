`timescale 1ns/1ps
//`include "std_iic.v"

module std_iic_master_tb;
    reg clk;
    reg rst_n;
    reg en_start;
    reg read_now;
    wire scl;
    wire sda;
    wire data_avalid;
    wire [7:0] data;
    
    // Instantiate the DUT (Device Under Test)
    std_iic_master uut (
        .clk(clk),
        .rst_n(rst_n),
        .en_start(en_start),
        .read_now(read_now),
        .scl(scl),
        .sda(sda), 
        .data_avalid(data_avalid),
        .data(data)
    );
    
    // Clock generation (50MHz)
    always #10 clk = ~clk;
    
    initial begin
        // Initialize signals
        clk = 0;
        rst_n = 1;
        en_start = 0;
        read_now = 0;
        
        // Reset sequence
        #50;
        rst_n = 0;
        #20;
        rst_n = 1;
        

        // Start MPU initialization
        #500;
        en_start = 1;
        #20;
        en_start = 0;
        
        // // Wait for initialization to complete
        // wait(init_done);
        
        // // Start data transfer
        // #5000;
        // read_now = 1;
        // #20;
        // read_now = 0;
        
        // // Monitor data
        // while (!data_avalid) #10;
        // wait(data_avalid);

        
        // Wait some cycles before ending the simulation
        #80000;
        $finish;
    end
    
endmodule

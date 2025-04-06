    `timescale 1ns/1ps
    `include "mpu.v"

    module mpu_tb;
        reg clk;
        reg rst_n;
        reg mpu_init;
        reg mpu_transfer;
        wire scl;
        wire sda;
        wire init_done;
        wire data_avalid;
        wire [7:0] data;
        wire busy_now;
        
        // Instantiate the DUT (Device Under Test)
        mpu uut (
            .clk(clk),
            .rst_n(rst_n),
            .mpu_init(mpu_init),
            .mpu_transfer(mpu_transfer),
            .scl(scl),
            .sda(sda),
            .init_done(init_done),
            .data_avalid(data_avalid),
            .data(data),
            .busy_now(busy_now)
        );
        
        // Clock generation (50MHz)
        always #10 clk = ~clk;
        
        initial begin
            // Initialize signals
            clk = 0;
            rst_n = 1;
            mpu_init = 0;
            mpu_transfer = 0;
            
            // Reset sequence
            #50;
            rst_n = 0;
            #20;
            rst_n = 1;
            

            // Start MPU initialization
            #100;
            mpu_init = 1;
            #20;
            mpu_init = 0;
            
            // Wait for initialization to complete
            wait(init_done);
            
            // Start data transfer
            #5000;
            mpu_transfer = 1;
            #20;
            mpu_transfer = 0;
            
            // Monitor data
            //while (!data_avalid) #10;
            wait(data_avalid);

            
            // Wait some cycles before ending the simulation
            #500;
            $finish;
        end
        
    endmodule

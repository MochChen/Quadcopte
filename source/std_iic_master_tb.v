`timescale 1ns/1ps
//`include "std_iic_master.v"

module std_iic_master_tb;
    reg clk;
    reg rst_n;
    reg en_start;
    reg [2:0] n_send;
    reg [2:0] m_read;
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
        .n_send(n_send),
        .m_read(m_read),
        .send_done(send_done),
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
        read_now = 1;
        
        // Reset sequence
        #50;
        rst_n = 0;
        #20;
        rst_n = 1;
        

        // Start MPU initialization
        #500;
        en_start = 1;
        n_send = 1; //2代表读2字节
        m_read = 1; //1代表读2字节
        #20;
        en_start = 0;
        n_send = 0;
        m_read = 0;
    

        
        // Wait some cycles before ending the simulation
        #80000;
        $finish;
    end
    
endmodule

`timescale 1ns/1ps

module mpu_top_tb;

    reg clk;
    reg rst_n;
    reg init_start;
    reg read_start;
    wire init_done;
    wire read_done;
    wire data_avalid;
    wire [7:0] data;
    wire scl;
    wire sda_wire;
    wire sda;  // inout绑定模拟

    assign sda = sda_wire;

    // Instantiate the top module
    mpu_top dut (
        .clk(clk),
        .rst_n(rst_n),
        .init_start(init_start),
        .read_start(read_start),
        .init_done(init_done),
        .read_done(read_done),
        .data_avalid(data_avalid),
        .data(data),
        .scl(scl),
        .sda(sda_wire)
    );

    // I2C slave 模拟器 (简单版本，仅测试级别)
    assign sda_wire = 1'bz; // 暂时模拟器空白，或你也可以加入i2c slave仿真模块

    // 时钟生成
    initial clk = 0;
    always #10 clk = ~clk; // 50 MHz 时钟

    initial begin
        $dumpfile("mpu_top_tb.vcd");
        $dumpvars(0, mpu_top_tb);

        // 初始化
        rst_n = 1;
        init_start = 0;
        read_start = 0;
        #100;

        rst_n = 0;
        #20;
        rst_n = 1;
        #1000;

        // 开始初始化过程
        init_start = 1;
        #20;
        init_start = 0;

        // 等待初始化完成
        wait (init_done);
        #100;

        // 开始读数据
        read_start = 1;
        #20;
        read_start = 0;

        // 等待读取完成
        wait (read_done);
        #100;

        $display("Test completed!");
        $finish;
    end

endmodule

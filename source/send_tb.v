`timescale 1ns / 1ps

module send_tb;

    // 输入信号
    reg clk;                  // 系统时钟
    reg scl_posedge;          // SCL 上升沿信号
    reg scl_negedge;          // SCL 下降沿信号
    reg start;                // 发送使能信号
    reg [7:0] send_buffer [15:0]; // 要发送的 8 位数据
    reg [3:0] send_cnt;       // 发送字节数

    // 输出信号
    wire done;                // 发送完成信号
    wire ack_error;           // ACK 错误标志
    wire scl;                 // I2C 时钟线
    wire sda;                 // I2C 数据线（双向）

    // 内部信号用于仿真从设备行为
    reg sda_drive;            // 从设备驱动 SDA 的值
    reg sda_oe;               // 从设备 SDA 输出使能

    // SDA 双向控制
    assign sda = sda_oe ? sda_drive : 1'bz;

    // 实例化被测模块 (DUT)
    send uut (
        .clk(clk),
        .scl_posedge(scl_posedge),
        .scl_negedge(scl_negedge),
        .start(start),
        .done(done),
        .ack_error(ack_error),
        .send_buffer(send_buffer),
        .send_cnt(send_cnt),
        .scl(scl),
        .sda(sda)
    );

    // 时钟生成
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns 周期，100MHz
    end

    // SCL 信号生成（模拟 I2C 时钟，50kHz）
    initial begin
        scl_posedge = 0;
        scl_negedge = 0;
        forever begin
            #10000;              // 10us 高电平
            scl_posedge = 1;
            #10 scl_posedge = 0; // 短暂脉冲
            #10000;              // 10us 低电平
            scl_negedge = 1;
            #10 scl_negedge = 0; // 短暂脉冲
        end
    end

    // 测试序列
    initial begin
        // 初始化信号
        start = 0;
        send_cnt = 0;
        sda_drive = 0;
        sda_oe = 0;
        $display("Testbench started");

        // 初始化发送缓冲区
        send_buffer[0] = 8'hA5;
        send_buffer[1] = 8'h5A;
        send_buffer[2] = 8'hFF;
        send_buffer[3] = 8'h00;

        // 测试 1: 发送 1 个字节，ACK 成功
        #100;
        $display("Test 1: Send 1 byte with ACK");
        send_cnt = 0; // 发送 1 个字节 (send_cnt + 1)
        start = 1;
        #100 start = 0;
        @(posedge scl_posedge); // 等待发送开始
        repeat (8) @(posedge scl_negedge); // 等待 8 位数据发送
        @(negedge scl_negedge); // ACK 周期
        sda_oe = 1; sda_drive = 0; // 从设备发送 ACK
        #20 sda_oe = 0; // 释放 SDA
        @(posedge done);
        $display("Test 1 done: ack_error = %b", ack_error);

        // 测试 2: 发送 2 个字节，第二个字节 NACK
        #100;
        $display("Test 2: Send 2 bytes with NACK on second byte");
        send_cnt = 1; // 发送 2 个字节
        start = 1;
        #100 start = 0;
        @(posedge scl_posedge);
        repeat (8) @(posedge scl_negedge); // 第一个字节数据
        @(negedge scl_negedge); // 第一个 ACK
        sda_oe = 1; sda_drive = 0; // ACK
        #20 sda_oe = 0;
        repeat (8) @(posedge scl_negedge); // 第二个字节数据
        @(negedge scl_negedge); // 第二个 ACK
        sda_oe = 1; sda_drive = 1; // NACK
        #20 sda_oe = 0;
        @(posedge done);
        $display("Test 2 done: ack_error = %b", ack_error);

        // 测试 3: 发送 3 个字节，全程 ACK
        #100;
        $display("Test 3: Send 3 bytes with all ACK");
        send_cnt = 2; // 发送 3 个字节
        start = 1;
        #100 start = 0;
        @(posedge scl_posedge);
        repeat (3) begin
            repeat (8) @(posedge scl_negedge); // 每字节 8 位
            @(negedge scl_negedge); // ACK 周期
            sda_oe = 1; sda_drive = 0; // ACK
            #20 sda_oe = 0;
        end
        @(posedge done);
        $display("Test 3 done: ack_error = %b", ack_error);

        // 测试结束
        #100;
        $display("Testbench finished");
        $finish;
    end

    // 波形转储
    initial begin
        $dumpfile("tb_send.vcd");
        $dumpvars(0, tb_send);
    end

endmodule
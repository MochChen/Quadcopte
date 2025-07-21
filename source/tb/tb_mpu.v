`timescale 1ns/1ps

module tb_mpu;

    reg clk = 0;
    reg rst_n = 0;

    reg mpu_init_start = 0;
    reg mpu_read_start = 0;
    wire mpu_read_done;
    wire [111:0] mpu_data_pack;

    wire scl;
    wire sda_oe;
    wire sda_out;
    wire sda_in;

    reg [7:0] mpu_device_addr     = 8'hD0; // 7bit地址左移+读写位
    reg [7:0] mpu_register_addr   = 8'h6B;
    reg [7:0] mpu_register_data   = 8'h00;
    reg [7:0] mpu_read_start_addr = 8'h3B;

    reg mpu_error_reset = 0;
    wire mpu_error;

    // 生成50MHz时钟
    always #10 clk = ~clk;

    // 简单模拟器：固定返回14字节递增数据（0x01 ~ 0x0E）
    reg [7:0] fake_data [0:13];
    initial begin
        fake_data[ 0] = 8'h01; fake_data[ 1] = 8'h02;
        fake_data[ 2] = 8'h03; fake_data[ 3] = 8'h04;
        fake_data[ 4] = 8'h05; fake_data[ 5] = 8'h06;
        fake_data[ 6] = 8'h07; fake_data[ 7] = 8'h08;
        fake_data[ 8] = 8'h09; fake_data[ 9] = 8'h0A;
        fake_data[10] = 8'h0B; fake_data[11] = 8'h0C;
        fake_data[12] = 8'h0D; fake_data[13] = 8'h0E;
    end

    // SDA模拟：你可以拓展成一个MPU响应器，这里简化处理，不驱动sda_in，保持为1
    wire sda = sda_oe ? sda_out : sda_in;

    mpu dut (
        .clk(clk),
        .rst_n(rst_n),

        .mpu_init_start(mpu_init_start),
        .mpu_read_start(mpu_read_start),
        .mpu_read_done(mpu_read_done),
        .mpu_data_pack(mpu_data_pack),

        .scl(scl),
        .sda_oe(sda_oe),
        .sda_out(sda_out),
        .sda_in(sda_in),

        .mpu_device_addr(mpu_device_addr),
        .mpu_register_addr(mpu_register_addr),
        .mpu_register_data(mpu_register_data),
        .mpu_read_start_addr(mpu_read_start_addr),

        .mpu_error_reset(mpu_error_reset),
        .mpu_error(mpu_error)
    );

    initial begin
        $display("Start TB");
        $dumpfile("tb_mpu.vcd");
        $dumpvars(0, tb_mpu);

        #100;
        rst_n = 1;

        // 模拟初始化写
        #100;
        mpu_init_start = 1;
        #20;
        mpu_init_start = 0;

        // 等一段时间再启动读取
        #500000;
        mpu_read_start = 1;
        #20;
        mpu_read_start = 0;

        // 等待读取完成
        wait (mpu_read_done);
        $display("MPU Data Read Done:");
        $display("mpu_data_pack = %h", mpu_data_pack);

        #200;
        $finish;
    end

endmodule

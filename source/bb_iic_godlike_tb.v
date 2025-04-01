`include "bb_iic_godlike.v"
module bb_iic_godlike_tb (
    input clk,          // 50 MHz
    input rst_n,
    input mpu_init,
    input mpu_transfer,
    output scl,
    inout sda,
    output [7:0] data,
    output data_avalid,
    output busy,
    output error
);
    bb_iic_godlike #(
        .CLK_MAIN(50_000_000),
        .SCL_FREQ(400_000),
        .TIMEOUT_CYCLES(10_000)
    ) u_iic (
        .clk(clk),
        .rst_n(rst_n),
        .scl(scl),
        .sda(sda),
        .mpu_init(mpu_init),
        .dev_addr(7'h68),   // MPU-6050 地址
        .reg_addr(8'h3B),   // 连续读起始寄存器
        .reg_data(8'h00),   // 初始化数据
        .init_done(),
        .mpu_transfer(mpu_transfer),
        .data_avalid(data_avalid),
        .data(data),
        .busy(busy),
        .error(error)
    );
endmodule
// `include "mpu_mid.v"
// `include "std_iic_master.v"

module mpu_top (
    input clk,
    input rst_n,

    input init_start,
    output init_done,
    input read_start,
    output read_done,
    
    output data_avalid,
    output [7:0] data,
    output scl,
    inout sda
);

    wire en_start;
    wire read_now;

    wire [15:0] n;
    wire [15:0] m;
    wire [15:0] data_packed;

    mpu_mid u_mpu_mid (
        .clk(clk),
        .rst_n(rst_n),
        .init_start(init_start),
        .read_start(read_start),
        .en_start(en_start),
        .rd_now(read_now),
        .n(n),
        .data_packed (data_packed),
        .m(m)
    );

    // 实例化 std_iic_master 模块
    std_iic_master #(
        .CLK_MAIN(50000000),   // 根据你的主时钟设定
        .SCL_DIV(800000),     // I2C SCL 分频
        .SLAVE_ADDR(8'h68)       // I2C 设备地址
    ) u_std_iic_master (
        .clk(clk),
        .rst_n(rst_n),
        .en_start(en_start),
        .n_send(n),
        .m_read(m),
        .send_done(init_done),
        .read_now(read_now),
        .scl(scl),
        .sda(sda), 
        .data_avalid(data_avalid),
        .data(data)
    );

    reg [3:0] cnt = 0;

    always @(posedge clk) begin
        if (cnt == 14)
            cnt <= 0;
        else if (data_avalid)
            cnt <= cnt + 1;
    end

    assign read_done = (cnt == 14);

endmodule
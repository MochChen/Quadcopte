//`include "iic.v"

module mpu (
    input  clk,
    input  rst_n,

    input  wire mpu_init_start,

    input  wire mpu_read_start,
    output reg mpu_read_done,
    output reg [111:0] mpu_data_pack,       // 一共是14byte

    output wire scl,
    output wire sda_oe,
    output wire sda_out,
    input  wire sda_in,

    input  wire [7:0] mpu_device_addr,
    input  wire [7:0] mpu_register_addr,
    input  wire [7:0] mpu_register_data,
    input  wire [7:0] mpu_read_start_addr,
    
    input  wire mpu_error_reset,
    output wire mpu_error
);

    reg start;
    reg read_now;
    
    reg [15:0] send_cnt;        // 发送2个字节,把iic 中的MAX_IIC_SEND_BYTE设置成2
    reg [15:0] read_cnt;
    reg [15:0] cmd_pack;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start <= 0;
            read_now <= 0;
            send_cnt <= 0;
            read_cnt <= 0;
            cmd_pack <= 0;

        end else begin
            if (mpu_init_start) begin
                start <= 1;
                read_now <= 0;
                send_cnt <= 1;  // 写2字节 = 寄存器地址 + 寄存器值
                read_cnt <= 0;  // 不作用
                cmd_pack[7:0]  <= mpu_register_addr;
                cmd_pack[15:8] <= mpu_register_data;

            end else if (mpu_read_start) begin
                start <= 1;
                read_now <= 1;
                send_cnt <= 0;  // 写1字节 = 读的起始地址
                read_cnt <= 13; // 读14字节
                cmd_pack[7:0]  <= mpu_read_start_addr;
                cmd_pack[15:8] <= 0;

            end else begin
                start <= 0;
                read_now <= 0;
                send_cnt <= 0;
                read_cnt <= 0;
                //cmd_pack <= 0;
            end
        end
    end

/**************************************************************************************\
                            实例化 std_iic_master 模块
\**************************************************************************************/ 
    wire iic_data_valid;
    wire [7:0] iic_data;
    wire iic_error_reset = mpu_error_reset;

    iic #(
        .CLK_MAIN(50000000),       // 50MHz 主时钟
        .SCL_DIV(800000),          // 400KHz是800K转换一次(tick)，500000000/800000 = 62.5
        .MAX_IIC_SEND_BYTE(2)
    ) ins_iic (
        .clk(clk),
        .rst_n(rst_n),

        .iic_start(start),
        .iic_read_now(read_now),
        .iic_send_cnt(send_cnt),
        .iic_read_cnt(read_cnt),
        .iic_cmd_pack(cmd_pack),

        .iic_data_valid(iic_data_valid),
        .iic_data(iic_data),

        .iic_error_reset(iic_error_reset),
        .iic_error(mpu_error),

        .scl(scl),
        .sda_oe(sda_oe),
        .sda_out(sda_out),
        .sda_in(sda_in),

        .slave_address(mpu_device_addr[6:0])
    );


    //输出字节计数
    reg [3:0] byte_cnt_14;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mpu_data_pack <= 0;
            byte_cnt_14 <= 0;
            mpu_read_done <= 0;
        end else begin
            if (iic_data_valid) begin
                mpu_data_pack[8*byte_cnt_14 +: 8] <= iic_data;
                byte_cnt_14 <= byte_cnt_14 + 1;
            end else if (byte_cnt_14 == 14) begin
                byte_cnt_14 <= 0;
                mpu_read_done <= 1;
            end else begin
                mpu_read_done <= 0;
            end
        end
    end

endmodule

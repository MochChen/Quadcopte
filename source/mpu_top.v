// `include "mpu_mid.v"
// `include "std_iic_master.v"

module mpu_top (
    input clk,
    input rst_n,

    input wire init_start,
    output wire init_done,
    input wire read_start,
    output wire read_done,
    
    output wire data_avalid,
    output wire [7:0] data,
    output wire scl,
    output wire sda_en,
    output wire sda_out,
    input wire sda_in,
    output wire  [2:0] iic_state
);

    wire en_start;
    wire read_now;

    wire [15:0] n;
    wire [15:0] m;
    wire [127:0] data_packed;

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
    std_iic_master u_std_iic_master (
        .clk(clk),
        .rst_n(rst_n),
        .en_start(en_start),
        .n_send(n),
        .m_read(m),
        .data_packed (data_packed),
        .init_done(init_done),
        .read_now(read_now),
        .scl(scl),
        .sda_en(sda_en),
        .sda_out(sda_out),
        .sda_in(sda_in),
        .data_avalid(data_avalid),
        .data(data),
        .iic_state(iic_state)
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

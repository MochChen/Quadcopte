module std_iic_master #(
    parameter SLAVE_ADDR = 7'h0B,
    parameter CLK_MAIN = 50000000, // 50MHz
    parameter SCL_DIV = 800000 // 400KHz是800K转换一次(tick)，500000000/800000 = 62.5
)(
    input clk,
    output [7:0] data_out,
    output data_avalid,

    output scl,
    inout sda
);
    // 相位累加器(dds)方法产生精确时钟, real 在 Verilog 中是 64 位 IEEE 754 双精度浮点数,防止溢出
    localparam real SCL_DIV_REAL = SCL_DIV;
    localparam real CLK_MAIN_REAL = CLK_MAIN;
    localparam real PHASE_INC_REAL = (SCL_DIV_REAL / CLK_MAIN_REAL) * (2.0 ** 32);
    localparam ACC_INC = $rtoi(PHASE_INC_REAL);//$rtoi 转换成整数,默认是32bit,注意ACC_INC不能太大

    reg [31:0] acc = 0; 
    wire [32:0] next_acc = acc + ACC_INC;
    always @(posedge clk) acc <= next_acc[31:0]; 
    wire tick = next_acc[32];
    reg scl_now = 1; always @(posedge clk) if (tick) scl_now <= ~scl_now;
    reg scl_pre = 1; always @(posedge clk) scl_pre <= scl_now;

    wire scl_posedge = ~scl_pre & scl_now;
    wire scl_negedge =  scl_pre & ~scl_now;

    // 写: 开始 + 设备地址 + (n*9bit) + 停止 ====== 开始 + n*9bit + 停止
    // 读: 开始 + 设备地址 + (n*9bit) + 开始 + 设备地址 + "{ ((m-1)*9bit) + 8bit + NACK + 停止 }"
    reg [7:0] slave_adr = SLAVE_ADDR;
    reg [3:0] n_cnt = 0; // n计数器
    reg [3:0] m_cnt = 0; // m计数器
    reg [7:0] tx_buffer [15:0] // 等待发送的数据,最大16个 = n_cnt;
    reg [7:0] rx_buffer [15:0] // 等待读的数据

endmodule




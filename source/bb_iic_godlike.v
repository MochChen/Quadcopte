module bb_iic_godlike #(
    parameter CLK_MAIN = 50_000_000,    // 主时钟频率 (50 MHz)
    parameter SCL_FREQ = 400_000,       // SCL 目标频率 (400 kHz)
    parameter TIMEOUT_CYCLES = 10_000   // 超时周期 (200 us @ 50 MHz)
) (
    input  wire clk,                    // 主时钟
    output wire scl,                    // I2C 时钟
    inout  wire sda,                    // I2C 数据
    input  wire rst_n,                  // 低有效复位
    input  wire mpu_init,               // MPU 初始化请求
    input  wire [6:0] dev_addr,         // 从设备地址 (动态输入)
    input  wire [7:0] reg_addr,         // 寄存器地址
    input  wire [7:0] reg_data,         // 写数据
    output reg  init_done,              // 初始化完成
    input  wire mpu_transfer,           // 连续传输请求
    output reg  data_avalid,            // 数据有效
    output reg  [7:0] data,             // 读取数据
    output wire busy,                   // 忙碌标志
    output reg  error                   // 错误标志 (超时/冲突)
);
    // 参数计算
    localparam PHASE_WIDTH = 32;
    localparam PHASE_INC = (SCL_FREQ << 1) * (1 << PHASE_WIDTH) / CLK_MAIN; // 2x SCL 频率
    localparam TIMEOUT_WIDTH = $clog2(TIMEOUT_CYCLES);

    // 状态编码：4 位压缩状态机
    localparam [3:0]
        IDLE      = 4'b0000,  // 空闲
        START     = 4'b0001,  // 起始
        SEND      = 4'b0010,  // 发送
        ACK_W     = 4'b0011,  // 写应答
        RESTART   = 4'b0100,  // 重复起始
        READ      = 4'b0101,  // 读取
        ACK_R     = 4'b0110,  // 读应答
        STOP      = 4'b0111;  // 停止

    // 寄存器
    reg [PHASE_WIDTH-1:0] phase_acc;    // 相位累加器
    reg [3:0] state;                    // 主状态
    reg [7:0] tx_buf [0:2];             // 发送缓冲区 (地址+寄存器+数据)
    reg [1:0] tx_ptr;                   // 发送指针
    reg [2:0] bit_ptr;                  // 位指针
    reg [2:0] rx_cnt;                   // 接收字节计数
    reg sda_out;                        // SDA 输出
    reg scl_gen;                        // SCL 生成
    reg init_flag;                      // 初始化标志
    reg forever_flag;                   // 连续读标志
    reg restart_flag;                   // 重启标志
    reg [TIMEOUT_WIDTH-1:0] timeout;    // 超时计数器

    // SCL 生成与边沿检测
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) phase_acc <= 0;
        else phase_acc <= phase_acc + PHASE_INC;
    end
    wire tick = phase_acc[PHASE_WIDTH-1];
    always @(posedge clk) scl_gen <= tick ? ~scl_gen : scl_gen;
    assign scl = (state != IDLE) ? scl_gen : 1'b1;
    reg [1:0] scl_edge; always @(posedge clk) scl_edge <= {scl_edge[0], scl_gen};
    wire scl_pos = scl_edge[1] & ~scl_edge[0];
    wire scl_neg = ~scl_edge[1] & scl_edge[0];

    // 超时与冲突检测
    wire bus_conflict = (state == SEND) & (sda_out & ~sda); // SDA 被拉低
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) timeout <= 0;
        else if (state == IDLE) timeout <= 0;
        else if (tick) timeout <= timeout + 1;
    end
    wire timeout_err = &timeout; // 全 1 表示超时

    // 主状态机
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            sda_out <= 1;
            init_done <= 0;
            data_avalid <= 0;
            init_flag <= 0;
            forever_flag <= 0;
            restart_flag <= 1;
            tx_ptr <= 0;
            bit_ptr <= 7;
            rx_cnt <= 0;
            error <= 0;
            tx_buf[0] <= 0;
            tx_buf[1] <= 0;
            tx_buf[2] <= 0;
        end
        else begin
            // 错误处理
            if (timeout_err | bus_conflict) begin
                state <= IDLE;
                error <= 1;
                sda_out <= 1;
            end
            else case (state)
                IDLE: begin
                    error <= 0;
                    sda_out <= 1;
                    if (mpu_init & ~init_flag) begin
                        state <= START;
                        init_flag <= 1;
                        tx_buf[0] <= {dev_addr, 1'b0}; // 写地址
                        tx_buf[1] <= 8'h6B;            // 寄存器
                        tx_buf[2] <= 8'h00;            // 数据
                        tx_ptr <= 2;
                    end
                    else if (mpu_transfer) begin
                        state <= START;
                        forever_flag <= 1;
                        tx_buf[0] <= {dev_addr, 1'b0}; // 写地址
                        tx_buf[1] <= reg_addr;         // 寄存器
                        tx_ptr <= 1;
                    end
                end
                START: if (scl_neg) begin
                    sda_out <= 0;
                    state <= SEND;
                    bit_ptr <= 7;
                end
                SEND: if (scl_neg) begin
                    if (~|bit_ptr) begin
                        state <= ACK_W;
                        sda_out <= 1;       // 释放 SDA
                    end
                    else begin
                        sda_out <= tx_buf[tx_ptr][bit_ptr];
                        bit_ptr <= bit_ptr - 1;
                    end
                end
                ACK_W: if (scl_pos) begin
                    if (sda == 0) begin     // 应答成功
                        if (~|tx_ptr) begin // 发送完成
                            state <= init_flag ? STOP : (restart_flag ? RESTART : READ);
                            tx_ptr <= 0;
                        end
                        else begin
                            tx_ptr <= tx_ptr - 1;
                            state <= SEND;
                            bit_ptr <= 7;
                        end
                    end
                    else error <= 1;        // 应答失败
                end
                RESTART: if (scl_neg) begin
                    sda_out <= 1;
                    if (scl_pos) begin
                        state <= START;
                        tx_buf[0] <= {dev_addr, 1'b1}; // 读地址
                        tx_ptr <= 0;
                        restart_flag <= 0;
                    end
                end
                READ: if (scl_pos) begin
                    data <= {data[6:0], sda};   // 移位接收
                    if (~|bit_ptr) state <= ACK_R;
                    else bit_ptr <= bit_ptr - 1;
                end
                ACK_R: if (scl_neg) begin
                    sda_out <= (rx_cnt == 7);   // 最后字节无应答
                    if (scl_pos) begin
                        data_avalid <= 1;
                        rx_cnt <= rx_cnt + 1;
                        bit_ptr <= 7;
                        if (&rx_cnt) state <= forever_flag ? RESTART : STOP;
                        else state <= READ;
                    end
                end
                STOP: if (scl_neg) begin
                    sda_out <= 0;
                    if (scl_pos) begin
                        sda_out <= 1;
                        state <= IDLE;
                        init_done <= init_flag;
                        forever_flag <= 0;
                        restart_flag <= 1;
                    end
                end
            endcase
            if (state != READ) data_avalid <= 0;
        end
    end

    // SDA 双向控制与状态输出
    assign sda = (state == SEND || state == ACK_R) ? sda_out : 1'bz;
    assign busy = (state != IDLE);
endmodule
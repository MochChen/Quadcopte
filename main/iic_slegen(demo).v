module i2c_master #(
    parameter CLK_FREQ = 50_000_000,  // 输入时钟频率 (50 MHz)
    parameter I2C_FREQ = 100_000,     // I2C 时钟频率 (100 kHz)
    parameter TIMEOUT_CYCLES = 1000   // 超时周期
)(
    input wire clk, rst,              // 时钟和复位
    input wire start,                 // 启动信号
    input wire [6:0] slave_addr,      // 7位从机地址
    input wire [7:0] data_in,         // 要发送的数据
    output reg scl,                   // I2C 时钟线
    inout wire sda,                   // I2C 数据线 (双向)
    output reg busy,                  // 忙碌标志
    output reg error                  // 错误标志
);
    // 参数计算
    localparam CLK_DIV = CLK_FREQ / (I2C_FREQ * 4); // SCL 每半周期的计数 (125)
    localparam STATE_WIDTH = 3;

    // 顶层状态定义
    localparam TOP_IDLE = 0, TOP_ACTIVE = 1, TOP_ERROR = 2;
    reg [STATE_WIDTH-1:0] top_state, next_top_state;

    // 子状态定义
    localparam SUB_START = 0, SUB_ADDR = 1, SUB_ACK1 = 2,
               SUB_DATA = 3, SUB_ACK2 = 4, SUB_STOP = 5;
    reg [STATE_WIDTH-1:0] sub_state, next_sub_state;

    // 内部信号
    reg [15:0] clk_cnt;               // SCL 分频计数器
    reg [3:0] bit_cnt;                // 位计数器 (地址+数据)
    reg [7:0] shift_reg;              // 移位寄存器
    reg sda_out;                      // SDA 输出值
    reg sda_oe;                       // SDA 输出使能 (1=输出, 0=高阻)
    wire sda_in = sda;                // SDA 输入值
    reg [15:0] timeout;               // 超时计数器

    // 双向 SDA 控制
    assign sda = sda_oe ? sda_out : 1'bz;

    // 顶层状态机
    always @(posedge clk or posedge rst) begin
        if (rst) top_state <= TOP_IDLE;
        else top_state <= next_top_state;
    end

    always @(*) begin
        case (top_state)
            TOP_IDLE:   next_top_state = start ? TOP_ACTIVE : TOP_IDLE;
            TOP_ACTIVE: next_top_state = (sub_state == SUB_STOP && clk_cnt == CLK_DIV-1) ? TOP_IDLE :
                                        (timeout == TIMEOUT_CYCLES) ? TOP_ERROR : TOP_ACTIVE;
            TOP_ERROR:  next_top_state = rst ? TOP_IDLE : TOP_ERROR;
            default:    next_top_state = TOP_IDLE;
        endcase
    end

    // 子状态机
    always @(posedge clk or posedge rst) begin
        if (rst || top_state != TOP_ACTIVE) sub_state <= SUB_START;
        else sub_state <= next_sub_state;
    end

    always @(*) begin
        case (sub_state)
            SUB_START: next_sub_state = (clk_cnt == CLK_DIV-1) ? SUB_ADDR : SUB_START;
            SUB_ADDR:  next_sub_state = (bit_cnt == 8 && clk_cnt == CLK_DIV-1) ? SUB_ACK1 : SUB_ADDR;
            SUB_ACK1:  next_sub_state = (clk_cnt == CLK_DIV-1) ? (sda_in ? TOP_ERROR : SUB_DATA) : SUB_ACK1;
            SUB_DATA:  next_sub_state = (bit_cnt == 8 && clk_cnt == CLK_DIV-1) ? SUB_ACK2 : SUB_DATA;
            SUB_ACK2:  next_sub_state = (clk_cnt == CLK_DIV-1) ? (sda_in ? TOP_ERROR : SUB_STOP) : SUB_ACK2;
            SUB_STOP:  next_sub_state = SUB_STOP; // 由顶层退出
            default:   next_sub_state = SUB_START;
        endcase
    end

    // 时钟分频和 SCL 生成
    always @(posedge clk or posedge rst) begin
        if (rst || top_state == TOP_IDLE) begin
            clk_cnt <= 0;
            scl <= 1;
        end
        else if (top_state == TOP_ACTIVE) begin
            if (clk_cnt == CLK_DIV-1) begin
                clk_cnt <= 0;
                scl <= ~scl; // SCL 翻转
            end
            else clk_cnt <= clk_cnt + 1;
        end
    end

    // 移位和位计数
    always @(posedge clk or posedge rst) begin
        if (rst || top_state == TOP_IDLE) begin
            shift_reg <= 0;
            bit_cnt <= 0;
        end
        else if (top_state == TOP_ACTIVE) begin
            if (clk_cnt == CLK_DIV-1 && scl) begin // SCL 上升沿
                case (sub_state)
                    SUB_START: shift_reg <= {slave_addr, 1'b0}; // 地址 + 写位
                    SUB_ADDR:  if (bit_cnt < 8) begin
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_cnt <= bit_cnt + 1;
                    end
                    SUB_DATA:  if (bit_cnt == 0) shift_reg <= data_in;
                               else if (bit_cnt < 8) begin
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_cnt <= bit_cnt + 1;
                    end
                endcase
            end
        end
    end

    // SDA 输出控制
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sda_out <= 1;
            sda_oe <= 0;
            busy <= 0;
            error <= 0;
        end
        else begin
            busy <= (top_state != TOP_IDLE);
            error <= (top_state == TOP_ERROR);
            case (sub_state)
                SUB_START: begin
                    if (clk_cnt == CLK_DIV/2) begin
                        sda_out <= 0; // 起始位：SDA 从 1 拉低
                        sda_oe <= 1;
                    end
                end
                SUB_ADDR, SUB_DATA: begin
                    if (clk_cnt == CLK_DIV-1 && !scl) sda_out <= shift_reg[7];
                    sda_oe <= 1;
                end
                SUB_ACK1, SUB_ACK2: begin
                    sda_oe <= 0; // 高阻，等待从机应答
                end
                SUB_STOP: begin
                    if (clk_cnt == CLK_DIV/2) sda_out <= 0;
                    if (clk_cnt == CLK_DIV-1) begin
                        sda_out <= 1; // 停止位：SDA 从 0 拉高
                        sda_oe <= 1;
                    end
                end
            endcase
        end
    end

    // 超时计数
    always @(posedge clk or posedge rst) begin
        if (rst || top_state != TOP_ACTIVE) timeout <= 0;
        else if (sub_state == SUB_ACK1 || sub_state == SUB_ACK2) timeout <= timeout + 1;
        else timeout <= 0;
    end

    // 调试状态名（仿真用）
    `ifdef SIMULATION
    reg [63:0] top_state_name, sub_state_name;
    always @(top_state) begin
        case (top_state)
            TOP_IDLE:   top_state_name = "IDLE";
            TOP_ACTIVE: top_state_name = "ACTIVE";
            TOP_ERROR:  top_state_name = "ERROR";
        endcase
    end
    always @(sub_state) begin
        case (sub_state)
            SUB_START: sub_state_name = "START";
            SUB_ADDR:  sub_state_name = "ADDR";
            SUB_ACK1:  sub_state_name = "ACK1";
            SUB_DATA:  sub_state_name = "DATA";
            SUB_ACK2:  sub_state_name = "ACK2";
            SUB_STOP:  sub_state_name = "STOP";
        endcase
    end
    `endif
endmodule
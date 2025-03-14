module exercise_elegant_sm #(
    parameter CLK_FREQ = 50_000_000,    // main frequence 50MHz
    parameter I2C_FREQ = 400_000 * 2,   // iic frequence 400KHz
    parameter TIMEOUT_CYCLES = 1000     // 超时周期,单位是clk，根据DDS进行计算具体值
)(
    input clk, rst,

    output wire  scl,           // I2C 时钟 
    inout  wire sda,            // I2C 数据

    input  wire mpu_init,       // 初始化
    input  wire mpu_transfer,   // 连续数据
    output reg data_avalid, 
    output reg [7:0] data,
    output reg busy_now
);
    // parameter calculation
    // test 02
    localparam STATE_WIDTH = 4;

    // internal signal
    wire sda_in = sda;

    typedef struct {
        reg [7:0] commands [7:0]; // 最多存储 8 个 8-bit 命令
        reg [2:0] num_bytes;      // 记录剩余命令个数，最多8个，0表示第一个
    } buffer_t;
    buffer_t cmd_buffer; // 命令缓存器                

    // sda
    assign sda = sda_oe ? sda_out : 1'bz;

    // scl
    localparam real CLK_FREQ_REAl = CLK_FREQ;
    localparam real I2C_FREQ_REAl = I2C_FREQ;
    localparam real PHASE_INC_REAL = (I2C_FREQ_REAl / CLK_FREQ_REAl) * (2.0 ** 32);
    localparam ACC_INC = $rtoi(PHASE_INC_REAL);

    reg [32:0] acc = 0;
    always @(posedge clk) if acc <= (top_state == TOP_ACTIVE) ? acc[31:0] + ACC_INC : 0;

    wire scl_gen = ~acc[32];
    assign scl = (top_state == TOP_ACTIVE) ? scl_gen : 1'b1;

    reg scl_gen_d = 1; always @(posedge clk) scl_gen_d <= scl_gen;
    wire scl_neg = scl_gen_d & ~scl_gen;
    wire scl_pos = ~scl_gen_d & scl_gen;


    // top state machine
    parameter TOP_IDLE = 0, TOP_ACTIVE = 1, TOP_ERROR = 2;
    reg [STATE_WIDTH - 1: 0] top_state, next_top_state;

    always @(posedge clk or posedge rst) begin
        if (rst) top_state <= TOP_IDLE;
        else top_state <= next_top_state;
    end

    always @(*) begin
        case (top_state)
            TOP_IDLE: next_top_state <= (mpu_init || mpu_transfer) ? TOP_ACTIVE : TOP_IDLE;
            TOP_ACTIVE: next_top_state <= (sub_state == SUB_STOP && scl_gen) ? TOP_IDLE :
                                        (timeout == TIMEOUT_CYCLES) ? TOP_ERROR : TOP_ACTIVE;
            TOP_ERROR: next_top_state = rst ? TOP_IDLE : TOP_ERROR;
            default: next_top_state = TOP_IDLE;
        endcase
    end

    // sub state machine
    localparam SUB_START = 0, SUB_SEND_BYTE = 1, SUB_R_ACK = 2,SUB_REMAIN_BYTE = 3,
               SUB_ARB_1 = 4, SUB_RESTART = 5, SUB_READ_BYTE = 6, SUB_W_ACK = 7,
               SUB_ABR_2 = 8, SUB_STOP = 9;
    reg [STATE_WIDTH - 1: 0] sub_state, next_sub_state;

    always @(posedge clk or posedge rst) begin
        if (rst || top_state != TOP_ACTIVE) sub_state <= SUB_START;
        else sub_state <= next_sub_state;
    end

    always @(*) begin
        case (sub_state)
            SUB_START: next_sub_state <= scl_neg ? SUB_SEND_BYTE : SUB_START;
            SUB_SEND_BYTE: next_sub_state <= (bit_cnt == 8 && scl_neg) ? SUB_R_ACK : SUB_SEND_BYTE;
            SUB_R_ACK:begin
                if (scl_pos) begin
                    if (sda_in) 
                        top_state <= TOP_ERROR;
                    else 
                        next_sub_state <= SUB_REMAIN_BYTE;
                end
                else begin
                    next_sub_state <= SUB_R_ACK;
                end
            end
            SUB_REMAIN_BYTE:begin
                if (scl_neg) begin
                    if (byte_cnt == cmd_buffer.num_bytes) 
                        next_sub_state <= SUB_ARB_1; // 发送完所有字节
                    else 
                        next_sub_state <= SUB_SEND_BYTE; // 继续发送字节
                end
                else begin
                    next_sub_state <= SUB_REMAIN_BYTE;
                end
            end
            // 在下降沿占用一个clk进行仲裁：
            SUB_ARB_1:begin
                if (initializing) begin // 判断是否在执行初始化，否则在执行连续读
                    state <= SUB_STOP;
                end 
                else if (first_restart) begin // 判断是否是否第一次restart
                    state <= SUB_RESTART; // 接收器会在时钟下降沿释放sda，此时才允许RSTART
                end
                else begin
                    state <= SUB_READ_BYTE;
                end
            end
            SUB_RESTART: next_sub_state <= scl_neg ? SUB_SEND_BYTE : SUB_RESTART;
            SUB_READ_BYTE: next_sub_state <= (bit_cnt == 8 && scl_neg) ? SUB_W_ACK : SUB_READ_BYTE;
            SUB_W_ACK:begin
                if (scl_neg) begin
                    if (byte_cnt == 12) begin
                        next_sub_state <= SUB_ABR_2;
                    end
                    else begin
                        next_sub_state <= SUB_READ_BYTE;
                    end
                end
            end
            // 在下降沿占用一个clk进行仲裁：
            SUB_ABR_2:begin
                if (forever_read) begin
                    next_sub_state <= RESTART;
                end
                else begin
                    next_sub_state <= SUB_STOP;
                end
            end
            SUB_STOP: next_sub_state <= SUB_STOP; // 顶层退出
            default: next_sub_state <= SUB_START;
        endcase
    end

    // 移位和位计数
    always @(posedge clk or posedge rst) begin
        if (rst || top_state == TOP_IDLE) begin
            
        end
        else if (top_state == TOP_ACTIVE) begin
            case (sub_state)
                SUB_START: sda_gen <= 0;
                SUB_SEND_BYTE: begin
                    if (scl_neg) begin
                        bit_cnt <= (bit_cnt == 8) ?  bit_cnt + 1 : 0;
                        sda_gen <= cmd_buffer.commands[byte_cnt][bit_cnt]; // 时钟下降沿改变sda数据
                    end
                end
                SUB_REMAIN_BYTE:begin
                    if (scl_neg) begin
                        byte_cnt <= (byte_cnt == cmd_buffer.num_bytes) ? 0 : byte_cnt + 1;
                    end
                end
            endcase
        end
    end



    // 超时计数
    always @(posedge clk or posedge rst) begin
        if (rst || top_state != TOP_ACTIVE) timeout <= 0;
        else if (sub_state == SUB_R_ACK || sub_state == SUB_W_ACK) timeout <= timeout + 1;
        else timeout <= 0;
    end






endmodule
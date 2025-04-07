//`define SIMULATION

module mpu #(
    parameter CLK_MAIN = 50000000, // 50MHz
    parameter SCL_DIV = 800000 // 400KHz是800K转换一次(tick)，500000000/800000 = 62.5
    ) 
    (
    input  wire clk,        // FPGA 主时钟 50MHz
    output wire  scl,        // I2C 时钟 
    inout  wire sda,        // I2C 数据
    input  wire rst_n,      // 复位信号（低有效）
    input  wire mpu_init,      // 初始化
    output reg init_done = 0,
    input  wire mpu_transfer, // 连续数据
    output reg data_avalid,
    output reg [7:0] data = 0,
    output wire busy_now
);

    reg [7:0] commands[0:7]; // 最多存储 8 个 8-bit 命令
    reg [2:0] num_bytes;      // 记录剩余命令个数，最多8个，0表示第一个

    localparam 
        IDLE        = 3'h0, 
        START       = 3'h1, 
        SEND_CYCLE  = 3'h2, 
        ARBITR_1    = 3'h3,  
        RESTART     = 3'h4, 
        READ_CYCLE  = 3'h5, 
        ARBITR_2    = 3'h6, 
        STOP        = 3'h7; 
    reg [2:0] state = IDLE; // 状态机

    localparam
        SEND_BYTE   = 2'h0, 
        R_ACK       = 2'h1, 
        REMAIN_BYTE = 2'h2;  
    reg [1:0] sub_state_send = SEND_BYTE; // send状态机

    localparam
        READ_BYTE   = 2'h0, 
        W_ACK       = 2'h1;
    reg sub_state_read = READ_BYTE; // read状态机

    // for scl:
    // 相位累加器(dds)方法产生精确时钟, real 在 Verilog 中是 64 位 IEEE 754 双精度浮点数,防止溢出
    localparam real SCL_DIV_REAL = SCL_DIV;
    localparam real CLK_MAIN_REAL = CLK_MAIN;
    localparam real PHASE_INC_REAL = (SCL_DIV_REAL / CLK_MAIN_REAL) * (2.0 ** 32);
    localparam ACC_INC = $rtoi(PHASE_INC_REAL);//$rtoi 转换成整数,默认是32bit,注意ACC_INC不能太大

    reg [31:0] acc = 0; 
    wire [32:0] next_acc = acc + ACC_INC;

    always @(posedge clk) acc <= next_acc[31:0]; 
    wire tick = next_acc[32];

    reg scl_gen = 1;
    always @(posedge clk) scl_gen <= tick ? ~scl_gen : scl_gen;

    reg scl_pre = 1; always @(posedge clk) scl_pre <= scl_gen;

    wire scl_pos = ~scl_pre & scl_gen;
    wire scl_neg =  scl_pre & ~scl_gen;


    reg scl_pos_delay; always @(posedge clk) scl_pos_delay <= scl_pos;
    reg scl_neg_delay; always @(posedge clk) scl_neg_delay <= scl_neg;

    wire scl_en = !(state == IDLE);
    assign scl = scl_en ? scl_gen : 1;

    // for sda:
    reg sda_gen = 1; //用于状态的sda产生
    reg initializing = 0; // 表示是否正在初始化
    reg [3:0] send_bit_cnt = 0;
    reg [3:0] send_byte_cnt = 0;
    reg [3:0] read_bit_cnt = 0;
    reg [3:0] read_byte_cnt = 0;
    reg first_restart = 1;
    reg forever_read = 0;
    reg error = 0;

    // 写: 开始 + 设备地址 + (n*9bit) + 停止 ====== 开始 + n*9bit + 停止
    // 读: 开始 + 设备地址 + (n*9bit) + 重新开始 + 设备地址 + (m-1)*9bit + 8bit + NACK + 停止
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            forever_read <= 0;
            data_avalid <= 0;
            sda_gen <= 1;
            error = 0;
            $display("  [MPU] 复位成功");
        end else begin
            case (state)
                IDLE: begin
                    if (mpu_init && !error) begin
                        initializing <= 1;
                        num_bytes <= 3'd2;
                        commands[0] <= 8'hD0;
                        commands[1] <= 8'h6B;
                        commands[2] <= 8'h00;
                        state <= START;
                        $display("  [MPU] 进入初始化...");
                    end else if (mpu_transfer && !error) begin
                        num_bytes <= 3'd1;
                        commands[0] <= 8'hD0;
                        commands[1] <= 8'h3B;
                        forever_read <= 1;
                        state <= START;
                        $display("  [MPU] 进入连续读...");
                    end
                end
                
                START: begin
                    if (scl_gen) sda_gen <= 0;
                    if (scl_neg) begin
                        state <= SEND_CYCLE;
                        $display("  [MPU] START");
                    end
                end
                
                SEND_CYCLE: begin
                    case (sub_state_send)
                        SEND_BYTE: begin
                            if (scl_neg && send_bit_cnt == 8) begin
                                send_bit_cnt <= 0;
                                sub_state_send <= R_ACK;
                                $display("  [MPU] send 8_bit...");
                            end else if (scl_neg) begin
                                send_bit_cnt <= send_bit_cnt + 1;
                                sda_gen <= commands[send_byte_cnt][send_bit_cnt];
                            end
                        end
                        R_ACK: begin
                            if (scl_pos) begin
                                if (1) begin
                                    sub_state_send <= REMAIN_BYTE;
                                    $display("  [MPU] ack true.");
                                end else begin
                                    state <= IDLE;
                                    error <= 1; 
                                    sub_state_send <= SEND_BYTE;
                                    $display("  [MPU] ack false.");
                                end
                            end
                        end
                        REMAIN_BYTE: begin
                            if (scl_neg) begin
                                if (send_byte_cnt == num_bytes) begin
                                    send_bit_cnt <= 0;
                                    send_byte_cnt <= 0;
                                    state <= ARBITR_1;
                                    sub_state_send <= SEND_BYTE;
                                    $display("  [MPU] 所有字节发送完");
                                end else begin
                                    send_bit_cnt <= 0;
                                    send_byte_cnt <= send_byte_cnt + 1;
                                    sub_state_send <= SEND_BYTE;
                                    $display("  [MPU] 还有字节未发送");
                                end
                            end
                        end
                        default: sub_state_send <= SEND_BYTE;
                    endcase
                end
                
                ARBITR_1: begin
                    if (initializing) begin
                        initializing <= 0;
                        init_done <= 1;
                        sda_gen <= 0;
                        state <= STOP;
                        $display("  [MPU] 初始化完成");
                    end else if (first_restart) begin
                        if (scl_pos) begin
                            sda_gen <= 1;
                            num_bytes <= 3'd0;
                            commands[0] <= 8'hD1;
                            first_restart <= 0;
                            state <= RESTART;
                            $display("  [MPU] 进入restart...");
                        end
                    end else begin
                        state <= READ_CYCLE;
                        $display("  [MPU] 进入读循环...");
                    end
                end
                
                RESTART: begin
                    if (scl_gen) sda_gen <= 0;
                    if (scl_neg) begin
                        state <= SEND_CYCLE;
                        $display("  [MPU] RESTART");
                    end
                end
                
                READ_CYCLE: begin
                    case (sub_state_read)
                        READ_BYTE: begin
                            data_avalid <= 0;
                            if (scl_neg &&  read_bit_cnt == 8) begin
                                sda_gen <= 1;
                                sub_state_read <= W_ACK;
                                //$display("  read 8_bit...");
                                $display("  [MPU] 读取到字节: %h", data);
                            end
                            if (scl_pos) begin
                                `ifdef SIMULATION
                                    data <= $random & 8'hFF; // 用掩码提取低 8 位
                                    //data <= {data[6:0], ($random & 1'b1)}; // 只有 1 位随机
                                `else
                                    data <= {data[6:0], 1'b1};
                                `endif
                                read_bit_cnt <= read_bit_cnt + 1;
                            end
                        end
                        W_ACK: begin
                            sda_gen <= 0;
                            // 判断还有没有需要读取的字节,一共读取12byte
                            if (scl_neg) begin
                                if (read_byte_cnt == 11) begin
                                    read_bit_cnt <= 0;
                                    read_byte_cnt <= 0;
                                    sda_gen <= 1;
                                    data_avalid <= 1;
                                    state <= ARBITR_2;
                                    sub_state_read <= READ_BYTE;
                                    $display("  [MPU] send nack.");
                                end else begin
                                    data_avalid <= 1;
                                    read_byte_cnt <= read_byte_cnt + 1;
                                    read_bit_cnt <= 0;
                                    sub_state_read <= READ_BYTE;
                                    $display("  [MPU] send ack.");
                                end
                            end
                        end
                    endcase
                end
                
                ARBITR_2: begin
                    data_avalid <= 0;
                    if (forever_read) begin
                        num_bytes <= 3'd1;
                        commands[0] <= 8'hD0;
                        commands[1] <= 8'h3B;
                        first_restart <= 1;
                        state <= RESTART;
                        $display("  [MPU] 进入restart...");
                    end else begin
                        sda_gen <= 0;
                        state <= STOP;
                    end
                end
                
                STOP: begin
                    //if (scl_neg) state <= IDLE;
                    if (scl_pos_delay) begin
                        sda_gen <= 1;
                        state <= IDLE;
                        $display("  [MPU] STOP");
                    end   
                end
                
                default: state <= IDLE;
            endcase
        end
    end

    wire sda_permit_write = (state == START) || 
                            ((state == SEND_CYCLE) && (sub_state_read == SEND_BYTE)) ||
                            (state == RESTART) ||
                            ((state == READ_CYCLE) && (sub_state_read == W_ACK)) ||
                            (state == STOP);
    assign sda = sda_permit_write ? sda_gen : 1'b1;
    assign busy_now = !(state == IDLE);


endmodule

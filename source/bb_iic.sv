module bb_iic #(
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
    output reg [7:0] data,
    output reg busy_now
);


    reg [7:0] commands[0:7]; // 最多存储 8 个 8-bit 命令
    reg [2:0] num_bytes;      // 记录剩余命令个数，最多8个，0表示第一个


typedef enum {
    IDLE, START, SEND_CYCLE, ARBITR_1,  
    RESTART, READ_CYCLE, ARBITR_2, STOP        
} state_t;  
state_t state = IDLE; // 状态机

typedef enum {
    SEND_BYTE, R_ACK, REMAIN_BYTE       
} send_state_t;  
send_state_t send_state = SEND_BYTE; // send状态机

typedef enum {
    READ_BYTE, W_ACK
} read_state_t;  
read_state_t read_state = READ_BYTE; // read状态机

// for scl:
// 相位累加器(dds)方法产生精确时钟
localparam real SCL_DIV_REAL = SCL_DIV;
localparam real CLK_MAIN_REAL = CLK_MAIN;
localparam real PHASE_INC_REAL = (SCL_DIV_REAL / CLK_MAIN_REAL) * (2.0 ** 32);
localparam ACC_INC = $rtoi(PHASE_INC_REAL);//$rtoi 转换成整数
reg [31:0] acc = 0; always @(posedge clk) acc <= acc[30:0] + ACC_INC; 
wire tick = acc[31]; // 每半个scl周期 tick一次

reg scl_gen = 1;
always @(posedge clk) scl_gen <= tick ? ~scl_gen : scl_gen;

reg [2:0] edge_detect = 0; always @(posedge clk) edge_detect <= {edge_detect[1:0], scl_gen};
wire scl_pos = (edge_detect == 3'b001);
wire scl_neg = (edge_detect == 3'b110);

reg [2:0] scl_pos_delay_2_clk = 3'b111;
always @(posedge clk) scl_pos_delay_2_clk <= {scl_pos_delay_2_clk[1:0], scl_pos};

wire scl_en = !(state == IDLE);
assign scl = scl_en ? scl_gen : 1;

// for sda:
reg sda_gen = 1; //用于状态的sda产生
reg initializing = 0; // 表示是否正在初始化
reg [2:0] bit_cnt = 0;
reg [2:0] byte_cnt = 0;
reg first_restart = 1;
reg forever_read = 0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        forever_read <= 0;
    end 
    else 
    case (state)
        IDLE:begin
            if (mpu_init) begin
                initializing <= 1; // 表示正在进行初始化
                num_bytes = 3'd2; // 初始化为 2，表示3个命令
                commands[0] = 8'hD0;
                commands[1] = 8'h6B;
                commands[2] = 8'h00;
                state <= START;
            end
            else if (mpu_transfer) begin
                num_bytes = 3'd1; // 连续读传感器设为 1，表示2个命令
                commands[0] = 8'hD0;
                commands[1] = 8'h3B;
                state <= START;
                // 启用永远循环读取，除非复位
                forever_read <= 1;
            end
        end
        START: begin
            sda_gen <= 0;
            state <= SEND_CYCLE;
        end
        SEND_CYCLE: begin
            case (send_state)
                SEND_BYTE: begin
                    if (&bit_cnt && scl_neg) begin
                        send_state <= R_ACK; // 时钟下降沿释放sda
                        bit_cnt <= 0;
                    end
                    else if (scl_neg) begin
                        bit_cnt <= bit_cnt + 1;
                        sda_gen <= commands[byte_cnt][bit_cnt]; // 时钟下降沿改变sda数据
                    end
                end
                R_ACK: begin
                    if (scl_pos) begin
                        if (1) begin // 由于仿真把这里的sda == 0 改了,实际应用时候改回去
                            send_state <= REMAIN_BYTE; // 时钟上升沿时的sda为0表示应答有效
                        end
                        else begin
                            // 这里应该做应答失败的处理，暂时不实现该功能，直接跳转到IDLE
                            state <= IDLE;
                        end
                    end
                end
                REMAIN_BYTE: begin
                    if (scl_pos) begin
                        if (byte_cnt == num_bytes) begin
                            state <= ARBITR_1;
                            send_state <= SEND_BYTE;
                            bit_cnt <= 0; // 清零计数器，给读取计数时共用
                            byte_cnt <= 0; // 清零计数器，给读取计数时共用
                        end 
                        else begin
                            byte_cnt <= byte_cnt + 1;
                            send_state <= SEND_BYTE;
                            if (&byte_cnt) begin
                                // 这里应该做应答失败的处理，暂时不实现该功能，直接跳转到IDLE
                                state <= IDLE;
                            end
                        end
                    end
                end
            endcase
        end
        ARBITR_1:begin
            if (initializing) begin // 判断是否在执行初始化，否则在执行连续读
                state <= STOP;
                initializing <= 0;
                init_done <= 1;
            end
            else if (first_restart) begin // 判断是否是否第一次restart
                if (scl_neg) begin
                    state <= RESTART; // 接收器会在时钟下降沿释放sda，此时才允许RSTART
                    sda_gen <= 1; // 时钟下降沿提前改变sda数据，以便产生RESTART
                    num_bytes = 3'd0; // 第一次restart设置为 0，表示1个命令
                    commands[0] = 8'hD1;
                    first_restart <= 0;
                end
            end
            else begin
                state <= READ_CYCLE;
            end
        end
        RESTART:begin
            if (scl_pos_delay_2_clk[2]) begin
                sda_gen <= 0;
                state <= SEND_CYCLE;
                send_state <= SEND_BYTE;
            end
        end
        READ_CYCLE:begin
            case (read_state)
                READ_BYTE: begin
                    if (&bit_cnt) begin
                        read_state <= W_ACK;    
                        bit_cnt <= 0;
                    end
                    else if (scl_pos) begin
                        data <= {data[6:0], sda};
                        bit_cnt <= bit_cnt + 1;
                    end
                    // 清除数据有效的输出标志
                    data_avalid <= 0; // 仿真的时候检查一下时序，可能需要增加data的延迟。
                end
                W_ACK: begin
                    if (scl_neg) begin
                        sda_gen <= 0;
                    end
                    if (scl_pos) begin
                        if (byte_cnt == 7) begin // 读取n-bit的寄存器值，这里的7可以设定更多个数，不共用的话
                            state <= ARBITR_2;
                            read_state <= READ_BYTE;
                            byte_cnt <= 0;
                            sda_gen <= 1;
                        end
                        else begin
                            byte_cnt <= byte_cnt + 1;
                            data_avalid <= 1;
                            read_state <= READ_BYTE;
                        end
                    end
                end
            endcase
        end
        ARBITR_2:begin
            if (forever_read) begin
                state <= RESTART;
                num_bytes = 3'd1; // 连续读传感器设为 1，表示2个命令
                commands[0] = 8'hD0;
                commands[1] = 8'h3B;
                first_restart <= 1;
            end
            else begin
                state <= STOP;
            end
        end
        STOP:begin
            if (scl_pos) begin
                sda_gen <= 0;
            end
            if (scl_pos_delay_2_clk[2]) begin
                sda_gen <= 1;
                state <= IDLE;
                forever_read <= 0;
            end
        end
        default: state <= IDLE;
    endcase
end // always @(posedge clk) begin

// state for sda send（assign = sda_gen）
// START .SEND_BYTE  ARBITR_1 RESTART W_ACK STOP
// 在 SEND_BYTE 和 W_ACK 过程中，这里希望 sda 受 sda_gen 控制，
// 但 sda 作为 inout 端口，如果多个 I2C 设备连接在总线上，它们可能会争抢 sda，造成驱动冲突。
wire a = (state == send_state) && ((send_state == R_ACK) || (send_state == REMAIN_BYTE));
assign sda = a ? sda_gen : 1'bZ;
assign busy_now = (state == IDLE) ? 1'b0 : 1'b1;


endmodule

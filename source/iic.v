/**************************************************************************************\
readme:
    -   使用错误报警功能,当ack没有正确应答的时候,iic_error会被拉高,状态机进入ERROR状态,直到
        收到iic_data_valid的flag信号被复位,复位后进入IDLE;

\**************************************************************************************/
`define VSC_SIM

module iic #(
    parameter CLK_MAIN = 50000000,   // 50MHz 主时钟
    parameter SCL_DIV = 800000,     // 400KHz是800K转换一次(tick)，500000000/800000 = 62.5
    parameter MAX_IIC_SEND_BYTE = 2
)(
    input  wire clk, 
    input  wire rst_n,

    input  wire iic_start,
    input  wire iic_read_now,
    input  wire [15:0] iic_send_cnt,
    input  wire [15:0] iic_read_cnt,
    input  wire [MAX_IIC_SEND_BYTE * 8 - 1 : 0] iic_cmd_pack,  // 发送MAX_IIC_SEND_BYTE = 2个字节

    output reg iic_data_valid,
    output reg [7:0] iic_data,
    
    input wire iic_error_reset,
    output reg iic_error,

    output wire scl,
    output wire sda_oe,
    output wire sda_out,
    input  wire sda_in,

    input  wire [6:0] slave_address
    
);

/**************************************************************************************\
                                将命令转化成数组
\**************************************************************************************/
    wire [7:0] cmd_array [MAX_IIC_SEND_BYTE-1 : 0];
    genvar i;
    generate
        for (i = 0; i < MAX_IIC_SEND_BYTE; i = i + 1) begin
            assign cmd_array[i] = iic_cmd_pack[i*8 +:8];
        end
    endgenerate

/**************************************************************************************\
                                    scl时钟生成
\**************************************************************************************/

    // 相位累加器(dds)产生tick
    localparam real SCL_DIV_REAL = SCL_DIV;         // real 在Verilog中是 64 位双精度浮点数
    localparam real CLK_MAIN_REAL = CLK_MAIN;
    localparam real PHASE_INC_REAL = (SCL_DIV_REAL / CLK_MAIN_REAL) * (2.0 ** 32);
    localparam ACC_INC = $rtoi(PHASE_INC_REAL);     // $rtoi 转换成整数,默认是32bit

    reg [31:0] acc; 
    wire [32:0] next_acc = acc + ACC_INC;           // 注意ACC_INC不能超过1/4 * 2的32次方,最大1/4主频
    wire tick = next_acc[32];

    reg scl_en;     // 时钟启动信号
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc <= 0;
        end else begin
            acc <= scl_en ? next_acc[31:0] : 0;
        end
    end

    reg scl_shadow;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            scl_shadow <= 1;
        else if (tick) 
            scl_shadow <=  ~scl_shadow;
    end
    assign scl = scl_shadow;


    // scl 的"1/4位置信号",用于精准stop,精准拉高sda,精准sda赋值
    wire one_quarter_scl = (acc[31] && !acc[30]);  // 相位的一半
    
    reg pre_one_quarter_scl = 0; 
    always @(posedge clk) pre_one_quarter_scl <= one_quarter_scl; 

    wire one_quarter_scl_tick = (one_quarter_scl && !pre_one_quarter_scl);// "1/4位置信号"的上升沿

    reg one_quarter_scl_tick_delay = 0;
    always @(posedge clk) one_quarter_scl_tick_delay <= one_quarter_scl_tick;
    
/**************************************************************************************\
                                    sda 生成
\**************************************************************************************/
    localparam  IDLE    = 4'h0,
                START   = 4'h1,
                SEND    = 4'h2,
                R_ACK   = 4'h3,
                READ    = 4'h4,
                W_ACK   = 4'h5,
                RESTART = 4'h6,
                STOP    = 4'h7,
                ERROR   = 4'h8;
    reg [3:0] state = IDLE;
    reg [3:0] next_state = R_ACK;

    reg [7:0] data_shift;
    reg [3:0] bit_cnt_send;
    reg [3:0] bit_cnt_read;
    reg [16:0] var_send_cnt;
    reg [15:0] var_read_cnt;

    reg [16:0] CONST_SEND_CNT; // 缓存作为常量比较

    reg restart_once;
    reg read_now;

    reg sda_shadow;
    assign sda_oe = (state == START)    ||
                    (state == SEND)     ||
                    (state == STOP)     ||
                    (state == RESTART)  ||
                    (state == W_ACK);
    assign sda_out = sda_shadow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_shadow <= 1;
            scl_en <= 0;

            state <= IDLE;
            next_state <= R_ACK;

            bit_cnt_send <= 0;
            bit_cnt_read <= 0;
            var_send_cnt <= 0;
            var_read_cnt <= 0;
            CONST_SEND_CNT <= 0;

            restart_once <= 0;
            iic_data_valid <= 0;
            read_now <= 0;

            data_shift <= 0;
            iic_data <= 0;

            iic_error <= 0;

        end else case (state)
            IDLE: // state = 0
                begin
                    if (iic_start) begin
                        scl_en <= 1;                        // 时钟开始
                        read_now <= iic_read_now;

                        var_send_cnt <= iic_send_cnt + 1;   // 算上设备地址
                        var_read_cnt <= iic_read_cnt;
                        CONST_SEND_CNT <= iic_send_cnt + 1;     // 用于比较

                        sda_shadow <= 1;
                        state <= START;
                    end
                end
            START: // state = 1
                begin
                    if (scl_shadow && one_quarter_scl_tick) begin
                        data_shift <= {slave_address, 1'b0};
                        sda_shadow <= 0;
                    end
                    if (!scl_shadow && one_quarter_scl_tick) begin
                        state <= SEND;
                    end
                end
            SEND: // state = 2
                begin
                    sda_shadow <= data_shift[7 - bit_cnt_send];//MSB first

                    if (!scl_shadow && one_quarter_scl_tick) begin
                        if (bit_cnt_send == 7) begin
                            bit_cnt_send <= 0;
                            // 进ack不用管sda_shadow状态
                            state <= R_ACK;
                        end else begin
                            bit_cnt_send <= bit_cnt_send + 1;
                        end
                    end
                end
            R_ACK: // state = 3
                begin
                    if (scl_shadow && one_quarter_scl_tick) begin
                    `ifdef VSC_SIM
                        if (1) begin //仿真用
                    `else 
                        if (!sda_in) begin                  // ack正确
                    `endif 
                            if (var_send_cnt == 0) begin    // 字节已经发送完成
                                if (read_now) begin         // 当前是读
                                    if (restart_once) begin
                                        // 进READ不用管sda_shadow状态
                                        next_state <= READ;
                                    end else begin
                                        sda_shadow <= 1;
                                        next_state <= RESTART;
                                    end
                                end else begin              // 当前是写
                                    sda_shadow <= 0;
                                    next_state <= STOP;
                                end
                            end else begin              // 还有字节没发送
                                var_send_cnt <= var_send_cnt - 1;
                                data_shift <= cmd_array[CONST_SEND_CNT - var_send_cnt];
                                // 进send不用管sda_shadow状态
                                next_state <= SEND;
                            end
                        end else begin                  // ack错误
                            iic_error <= 1;
                            sda_shadow <= 0;
                            next_state <= STOP;
                        end
                    end

                    if (!scl_shadow && one_quarter_scl_tick) state <= next_state;
                end
            READ: // state = 4
                begin
                `ifdef VSC_SIM
                    if (scl_shadow && one_quarter_scl_tick) iic_data[7-bit_cnt_read] <= $urandom % 2; //仿真用
                `else
                    if (scl_shadow && one_quarter_scl_tick) iic_data[7-bit_cnt_read] <= sda_in; //高位在前，MSB first
                `endif
                    if (!scl_shadow && one_quarter_scl_tick) begin
                        if (bit_cnt_read == 7) begin
                            state <= W_ACK;
                            bit_cnt_read <= 0;
                            sda_shadow <= (var_read_cnt == 0) ? 1 : 0; // 最后一位NACK,其他ACK
                        end else begin
                            bit_cnt_read <= bit_cnt_read + 1;
                        end
                    end
                end
            W_ACK: // state = 5
                begin
                    iic_data_valid <= (scl_shadow && one_quarter_scl_tick) ? 1 : 0;

                    if (!scl_shadow && one_quarter_scl_tick) begin
                        if (var_read_cnt == 0) begin
                            sda_shadow <= 0;//停止前必须是0
                            state <= STOP;
                        end else begin
                            var_read_cnt <= var_read_cnt - 1;
                            state <= READ;
                        end
                    end
                end
            RESTART: // state = 6
                begin
                    if (scl_shadow && one_quarter_scl_tick) begin
                        data_shift <= {slave_address, 1'b1};
                        sda_shadow <= 0;
                    end
                    if (!scl_shadow && one_quarter_scl_tick) state <= SEND;
                    restart_once <= 1;
                end
            STOP: // state = 7
                begin
                    

                    if (scl_shadow && one_quarter_scl_tick) begin
                        scl_en <= 0;        // 时钟结束
                        sda_shadow <= 1;
                    end
                    if (scl_shadow && one_quarter_scl_tick_delay) begin
                        read_now <= 0;
                        restart_once <= 0;
                        // sda_shadow已经是1
                        state <= IDLE;
                    end
                end
            ERROR:
                begin
                    if (iic_error_reset) begin
                        iic_error <= 0;
                        state <= IDLE;
                    end
                end
            default: state <= IDLE;
        endcase
    end

endmodule
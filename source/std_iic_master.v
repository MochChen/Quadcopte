module std_iic_master #(
    parameter CLK_MAIN   = 50000000,   // 50MHz 主时钟
    parameter SCL_DIV    = 800000,     // 400KHz是800K转换一次(tick)，500000000/800000 = 62.5
    parameter [6:0] SLAVE_ADDR = 7'h68 // 默认的I2C设备地址
    ) 
    (
    input  wire clk,        // FPGA 主时钟 50MHz
    input  wire rst_n,      // 复位信号（低有效）
    input  wire en_start,   // 开始信号
    input wire [15:0] n_send,
    input wire [15:0] m_read,
    input wire [127:0] data_packed,  // 16 × 8 = 128 位.实际上这里的127可以参数化
    output wire init_done,
    input  wire read_now,   // read_now = 1表示当前是读,否则是写
    output wire scl,        // I2C 时钟 
    output wire sda_en,
    output wire sda_out,
    input wire sda_in,
    output reg  data_avalid,
    output reg [7:0] data,
    output wire [2:0] iic_state
    
);

    assign iic_state = state;//调试监控

    wire [7:0] data_array [15:0];
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : assign_data_array
            assign data_array[i] = data_packed[i*8 +:8];
        end
    endgenerate

    // for scl:

    // 相位累加器(dds)产生时钟, real 在 Verilog 中是 64 位 IEEE 754 双精度浮点数,防止溢出
    localparam real SCL_DIV_REAL = SCL_DIV;
    localparam real CLK_MAIN_REAL = CLK_MAIN;
    localparam real PHASE_INC_REAL = (SCL_DIV_REAL / CLK_MAIN_REAL) * (2.0 ** 32);
    localparam ACC_INC = $rtoi(PHASE_INC_REAL);//$rtoi 转换成整数,默认是32bit,注意ACC_INC不能太大

    reg [31:0] acc; 
    wire [32:0] next_acc = acc + ACC_INC;

    reg scl_en = 0; //时钟开启信号,acc会延迟scl_en一个时钟clk
    always @(posedge clk) begin
        if (!rst_n) 
            acc <= 0;
        else if (scl_en)
            acc <= next_acc[31:0]; 
        else 
            acc <= 0;
    end

    wire tick = next_acc[32];
    reg scl_gen;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            scl_gen <= 1;
        else 
            scl_gen <= tick ? ~scl_gen : scl_gen;
    end
    assign scl = scl_gen;


    // scl 的1/4位置信号,用于精准stop(拉高sda)和精准sda数据赋值
    wire one_quarter_scl = (acc[31] && !acc[30]);  // 使用高位标志检测相位的一半
    reg prev_condition; 
    always @(posedge clk) prev_condition <= one_quarter_scl;
    wire one_quarter_scl_tick = (one_quarter_scl && !prev_condition);// 1/4位置信号的上升沿
    reg one_quarter_scl_tick_relay = 0;
    always @(posedge clk) one_quarter_scl_tick_relay <= one_quarter_scl_tick;
    
    // for sda:
    reg sda_gen;
    assign sda_en = (state == START) || (state == SEND) || (state == STOP) ||
                   (state == RESTART) || (state == W_ACK);
    assign sda_out = sda_gen;

    reg [7:0] data_shift;

    parameter   IDLE    = 3'h0,
                START   = 3'h1,
                SEND    = 3'h2,
                R_ACK   = 3'h3,
                READ    = 3'h4,
                W_ACK   = 3'h5,
                RESTART = 3'h6,
                STOP    = 3'h7;
    reg [2:0] state = IDLE;
    reg [2:0] next_state = R_ACK;
    reg [3:0] bit_cnt_send = 0;
    reg [3:0] bit_cnt_read = 0;
    reg [16:0] n = 0; // 缓存变量
    reg [15:0] m = 0; // 缓存变量
    reg [16:0] N_SEND; // 缓存作为常量比较
    reg [15:0] M_READ; // 缓存作为常量比较
    reg restart_once = 0;
    reg read_now_reg = 0;
    reg error;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_en <= 0;
            sda_gen <= 1;
            state <= IDLE;
            next_state <= R_ACK;
            bit_cnt_send <= 0;
            bit_cnt_read <= 0;
            restart_once <= 0;
            data_avalid <= 0;
            read_now_reg <= 0;

            data_shift <= 0;
            data <= 0;
            error <= 0;

        end else case (state)
            IDLE: //0
                begin
                    read_now_reg <= 0;
                    if (en_start) begin
                        state <= START;
                        scl_en <= 1;
                        read_now_reg <= read_now;
                        n <= n_send + 1; // 算上设备地址
                        m <= m_read;
                        N_SEND <= {1'b0, n_send};
                        M_READ <= m_read;
                    end
                end
            START: //1
                begin
                    if (scl_gen && one_quarter_scl_tick) begin
                        data_shift <= {SLAVE_ADDR, 1'b0};
                        sda_gen <= 0;
                    end
                    if (!scl_gen && one_quarter_scl_tick) state <= SEND;
                end
            SEND: //2
                begin
                    sda_gen <= data_shift[7 - bit_cnt_send];//MSB first

                    if (!scl_gen && one_quarter_scl_tick) begin
                        if (bit_cnt_send == 7) begin
                            state <= R_ACK;
                            bit_cnt_send <= 0;
                        end else begin
                            bit_cnt_send <= bit_cnt_send + 1;
                        end
                    end
                end
            R_ACK://3
                begin
                    //停止/错误
                    //继续发送
                    //重新开始
                    //读取数据
                    if (scl_gen && one_quarter_scl_tick) begin
                        if (!sda_in) begin
                        //if (1) begin // 仿真用
                            if (n == 0) begin
                                if (read_now_reg) begin
                                    if (restart_once) begin
                                        next_state <= READ;
                                    end else begin
                                        sda_gen <= 1;
                                        next_state <= RESTART;
                                    end
                                end else begin
                                    next_state <= STOP;
                                    sda_gen <= 0;//停止前必须是0
                                    $display("  [提示] 发送完成");
                                end
                            end else begin
                                next_state <= SEND;
                                n <= n - 1;
                                data_shift <= data_array[N_SEND - (n - 1)];//包内低字节先发送,数据放到包的时候用正常的MSB
                                $display("  [提示] 还有 %h 字节未发送", n);
                            end
                        end else begin      // slave ack fasle
                            next_state <= STOP;
                            error <= 1;
                            sda_gen <= 0;//停止前必须是0
                            $display("  [error] slave ack fasle");
                        end
                    end

                    if (!scl_gen && one_quarter_scl_tick) state <= next_state;
                end
            STOP: //4
                begin
                    if (scl_gen && one_quarter_scl_tick) begin
                        scl_en <= 0;
                        sda_gen <= 1;
                    end
                    if (scl_gen && one_quarter_scl_tick_relay) state <= IDLE;
                    restart_once <= 0;
                end
            RESTART: //5
                begin
                    if (scl_gen && one_quarter_scl_tick) begin
                        data_shift <= {SLAVE_ADDR, 1'b1};
                        sda_gen <= 0;
                    end
                    if (!scl_gen && one_quarter_scl_tick) state <= SEND;
                    restart_once <= 1;
                end
            READ: //6
                begin
                    if (scl_gen && one_quarter_scl_tick) data <= {data[6:0], sda_in};
                    //if (scl_gen && one_quarter_scl_tick) data <= {data[6:0], 1'b1}; //仿真用

                    if (!scl_gen && one_quarter_scl_tick) begin
                        if (bit_cnt_read == 7) begin
                            state <= W_ACK;
                            bit_cnt_read <= 0;
                            sda_gen <= (m == 0) ? 1 : 0; // 最后一位NACK,其他ACK
                        end else begin
                            bit_cnt_read <= bit_cnt_read + 1;
                        end
                    end
                end
            W_ACK: //7
                begin
                    data_avalid <= (scl_gen && one_quarter_scl_tick) ? 1 : 0;

                    if (!scl_gen && one_quarter_scl_tick) begin
                        if (m == 0) begin
                            state <= STOP;
                            sda_gen <= 0;//停止前必须是0
                        end else begin
                            m <= m - 1;
                            state <= READ;
                        end
                    end
                end
            default: state <= IDLE;
        endcase
    end

    assign init_done = ((state == STOP) && scl_gen && one_quarter_scl_tick_relay && !read_now_reg && !error);

endmodule
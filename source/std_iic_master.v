// ===============================================================
// Module: std_iic_master (Standard I2C Master with DDS-based Clock Generation)
// Author: [Moch_en]
// Date:   [2025-04-08]
// License: 
//          Licensed under the MIT License – free for personal and academic use;
//          commercial use requires separate licensing.
// ---------------------------------------------------------------
// Description:
//    A lightweight and configurable I2C master controller utilizing
//    Direct Digital Synthesis (DDS) to generate the I2C clock (SCL).
//    This module supports standard write and read operations with 
//    automatic RESTART conditions for read-after-write sequences.
//
// ---------------------------------------------------------------
// Parameter Configuration:
//    - CLK_MAIN    : System clock frequency in Hz (e.g., 50_000_000)
//    - SCL_DIV     : SCL tick rate (CLK_MAIN / SCL_DIV ≈ SCL frequency)
//    - N_SEND      : Number of bytes to transmit (excluding address)
//    - M_READ      : Number of bytes to receive
//    - SLAVE_ADDR  : 7-bit I2C slave address (MSB-aligned, R/W bit appended internally)
//
// ---------------------------------------------------------------
// I2C Transaction Flow:
//
//    Write Sequence:
//        START + SLAVE_ADDR + (n * 9bits) + STOP
//   
//    Read Sequence:
//        START + SLAVE_ADDR + (n * 9bits) + RESTART + SLAVE_ADDR + (m-1)*9bits + 8bits + NACK + STOP
//
//    Notes:
//        - MSB first!!! 
//        - Both n and m are zero-based counters!!!
//
// ---------------------------------------------------------------
// I/O Ports:
//    - clk          : Input  | System clock
//    - rst_n        : Input  | Active-low asynchronous reset
//    - en_start     : Input  | Trigger a new I2C transaction
//    - read_now     : Input  | If high, triggers a read operation (with RESTART)
//    - scl          : Output | I2C clock line
//    - sda          : Inout  | I2C data line
//    - data         : Output | Data received from I2C slave
//    - data_avalid  : Output | Pulse-high when `data` is valid (one per byte)
//
// ---------------------------------------------------------------
// Notes:
//    - The frequency divider range has not been analytically bounded;
//      simulation is recommended before use. 
//      Recommended SCL frequency range (for 50 MHz system clock): 0.2 Hz ~ 3.1 MHz. 
//      Values beyond this range may cause DDS instability or I2C timing violation.
//    - Error handling mechanism is not implemented.
//    - `en_start` must be pulsed high to trigger a transaction.
//    - Ensure `en_start` remains low between successive operations.
//    - The module internally manages timing, RESTART, and STOP conditions.
//    - Designed for master-only, single-master I2C environments.
// ===============================================================

module std_iic_master #(
    parameter CLK_MAIN   = 50000000,   // 50MHz 主时钟
    parameter SCL_DIV    = 800000,     // 400KHz是800K转换一次(tick)，500000000/800000 = 62.5
    parameter [6:0] SLAVE_ADDR = 7'h68 // 默认的I2C设备地址
    ) 
    (
    input  wire clk,        // FPGA 主时钟 50MHz
    output wire scl,        // I2C 时钟 
    inout  wire sda,        // I2C 数据
    input  wire rst_n,      // 复位信号（低有效）
    input  wire en_start,   // 开始信号
    input wire [15:0] n_send,
    input wire [15:0] m_read,
    output wire send_done,
    input  wire read_now,   // read_now = 1表示当前是读,否则是写
    output reg  data_avalid,
    output reg [7:0] data = 0
);

    // for scl:

    // 相位累加器(dds)产生时钟, real 在 Verilog 中是 64 位 IEEE 754 双精度浮点数,防止溢出
    localparam real SCL_DIV_REAL = SCL_DIV;
    localparam real CLK_MAIN_REAL = CLK_MAIN;
    localparam real PHASE_INC_REAL = (SCL_DIV_REAL / CLK_MAIN_REAL) * (2.0 ** 32);
    localparam ACC_INC = $rtoi(PHASE_INC_REAL);//$rtoi 转换成整数,默认是32bit,注意ACC_INC不能太大

    reg [31:0] acc = 0; 
    wire [32:0] next_acc = acc + ACC_INC;

    reg scl_en = 0; //时钟开启信号,acc会延迟scl_en一个时钟clk
    always @(posedge clk) begin
        if (scl_en) acc <= next_acc[31:0]; 
        else acc <= 0;
    end

    wire tick = next_acc[32];
    reg scl_gen = 1;
    always @(posedge clk) scl_gen = tick ? ~scl_gen : scl_gen;
    assign scl = scl_gen;

    // scl 的1/4位置信号,用于精准stop(拉高sda)和精准sda数据赋值
    wire one_quarter_scl = (acc[31] && !acc[30]);  // 使用高位标志检测相位的一半
    reg prev_condition; always @(posedge clk) prev_condition <= one_quarter_scl;
    wire one_quarter_scl_tick = one_quarter_scl && !prev_condition;// 1/4位置信号的上升沿
    reg one_quarter_scl_tick_relay = 0; always @(posedge clk) one_quarter_scl_tick_relay <= one_quarter_scl_tick;
    
    // for sda:
    reg sda_gen = 1;
    wire sda_dir = (state == START) || (state == SEND) || (state == STOP) ||
                   (state == RESTART) || (state == W_ACK);
    assign sda = sda_dir ? sda_gen : 1'bZ;
    reg [7:0] data_shift = 0;
    always @(posedge clk) begin
        if ((state == START) && scl && one_quarter_scl_tick) data_shift <= {SLAVE_ADDR, 1'b0};
        if (state == RESTART) data_shift <= {SLAVE_ADDR, 1'b1};
    end
    reg [7:0] data_packed [15:0];

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
    reg [15:0] n = 0; // 缓存变量
    reg [15:0] m = 0; // 缓存变量
    reg [15:0] N_SEND; // 缓存作为常量比较
    reg [15:0] M_READ; // 缓存作为常量比较
    reg restart_once = 0;
    reg read_now_reg = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc <= 0;
            scl_en <= 0;
            scl_gen <= 1;
            sda_gen <= 1;
            state <= IDLE;
            next_state <= R_ACK;
            bit_cnt_send <= 0;
            bit_cnt_read <= 0;
            restart_once <= 0;
            data_avalid <= 0;
            data_packed[0] <= 8'h6E;//模拟RA
            data_packed[1] <= 8'h66;//模拟DATA
            read_now_reg <= 0;

        end else case (state)
            IDLE: 
                begin
                    read_now_reg <= 0;
                    if (en_start) begin
                        state <= START;
                        scl_en <= 1;
                        read_now_reg <= read_now;
                        n <= n_send + 1; // 算上设备地址
                        m <= m_read;
                        N_SEND <= n_send;
                        M_READ <= m_read;
                    end
                end
            START: 
                begin
                    if (scl && one_quarter_scl_tick) sda_gen <= 0;
                    if (!scl && one_quarter_scl_tick) state <= SEND;
                end
            SEND:
                begin
                    sda_gen <= data_shift[7 - bit_cnt_send];//MSB first

                    if (!scl && one_quarter_scl_tick) begin
                        if (bit_cnt_send == 7) begin
                            state <= R_ACK;
                            bit_cnt_send <= 0;
                        end else begin
                            bit_cnt_send <= bit_cnt_send + 1;
                        end
                    end
                end
            R_ACK:
                begin
                    //停止/错误
                    //继续发送
                    //重新开始
                    //读取数据
                    if (scl && one_quarter_scl_tick) begin
                      //if (sda == 0) begin // slave ack true
                        if (1) begin // 仿真一直正确
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
                                data_shift <= data_packed[N_SEND - (n - 1)];//包内低字节先发送,数据放到包的时候用正常的MSB
                                $display("  [提示] 还有 %h 字节未发送", n);
                            end
                        end else begin      // slave ack fasle
                            next_state <= STOP;
                            sda_gen <= 0;//停止前必须是0
                            $display("  [error] slave ack fasle");
                        end
                    end

                    if (!scl && one_quarter_scl_tick) state <= next_state;
                end
            STOP: 
                begin
                    if (scl && one_quarter_scl_tick) begin
                        scl_en <= 0;
                        sda_gen <= 1;
                    end
                    if (scl && one_quarter_scl_tick_relay) state <= IDLE;
                    restart_once <= 0;
                end
            RESTART:
                begin
                    if (scl && one_quarter_scl_tick) sda_gen <= 0;
                    if (!scl && one_quarter_scl_tick) state <= SEND;
                    restart_once <= 1;
                end
            READ:
                begin
                    if (scl && one_quarter_scl_tick) data <= {data[6:0], sda};

                    if (!scl && one_quarter_scl_tick) begin
                        if (bit_cnt_read == 7) begin
                            state <= W_ACK;
                            bit_cnt_read <= 0;
                            sda_gen <= (m == 0) ? 1 : 0; // 最后一位NACK,其他ACK
                        end else begin
                            bit_cnt_read <= bit_cnt_read + 1;
                        end
                    end
                end
            W_ACK:
                begin
                    data_avalid <= (scl && one_quarter_scl_tick) ? 1 : 0;

                    if (!scl && one_quarter_scl_tick) begin
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

    assign send_done = ((state == STOP) && scl && one_quarter_scl_tick_relay && !read_now_reg);

endmodule
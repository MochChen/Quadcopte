module send (
    input clk,                // 系统时钟
    input scl_posedge,        // SCL 上升沿信号
    input scl_negedge,        // SCL 下降沿信号
    input start,              // 发送使能信号，类似“调用”触发
    output reg done,          // 发送完成信号
    output reg ack_error,     // ACK 错误标志（NACK 导致的中止）
    input [7:0] send_buffer [15:0],     // 要发送的 8 位数据
    input [3:0] send_cnt,     // 发送字节数
    output scl,               // I2C 时钟线（透传或控制）
    inout sda                 // I2C 数据线
);

    // 内部信号
    reg [3:0] byte_cnt;       // 当前发送的字节计数器
    reg [3:0] bit_cnt;        // 当前字节内的位计数器
    reg [7:0] shift_reg;      // 移位寄存器，用于发送数据
    reg sda_out;              // SDA 输出值
    reg sda_oe;               // SDA 输出使能（控制 SDA 为输出还是高阻态）
    wire sda_in;              // SDA 输入值，用于读取 ACK

    // 状态机状态定义
    localparam IDLE = 2'b00,  // 空闲状态
               SEND = 2'b01,  // 发送状态
               DONE = 2'b10;  // 完成状态
    reg [1:0] state;

    // SCL 直接透传（假设外部提供）

    // SDA 双向控制
    assign sda = sda_oe ? sda_out : 1'bz;
    assign sda_in = sda;      // 读取 SDA 的输入值

    // 主逻辑
    always @(posedge clk) begin
        case (state)
            IDLE: begin
                done <= 1'b0;
                ack_error <= 1'b0;         // 初始化 ACK 错误标志
                byte_cnt <= 4'b0;
                bit_cnt <= 4'b0;
                sda_oe <= 1'b0;
                if (start) begin
                    state <= SEND;
                    shift_reg <= send_buffer[0]; // 加载第一个字节
                    sda_oe <= 1'b1;              // 使能 SDA 输出
                end
            end

            SEND: begin
                if (scl_negedge) begin // 在 SCL 下降沿更新数据
                    if (bit_cnt < 8) begin
                        sda_out <= shift_reg[7];       // 输出最高位
                        shift_reg <= {shift_reg[6:0], 1'b0}; // 左移
                        bit_cnt <= bit_cnt + 1;
                    end else if (bit_cnt == 8) begin // 第 9 个周期（ACK 位）
                        sda_oe <= 1'b0;                // 释放 SDA，等待从设备应答
                        bit_cnt <= bit_cnt + 1;
                    end
                end else if (scl_posedge && bit_cnt == 9) begin // 在 SCL 上升沿检查 ACK
                    if (sda_in == 1'b0) begin // ACK 有效（从设备拉低 SDA）
                        if (byte_cnt < send_cnt) begin
                            byte_cnt <= byte_cnt + 1;
                            shift_reg <= send_buffer[byte_cnt + 1]; // 加载下一个字节
                            bit_cnt <= 0;
                            sda_oe <= 1'b1;            // 重新使能 SDA 输出
                        end else begin
                            state <= DONE;             // 所有字节发送完成
                            ack_error <= 1'b0;         // 成功完成，无 ACK 错误
                        end
                    end else begin // NACK（从设备未拉低 SDA）
                        state <= DONE;                 // 提前结束
                        ack_error <= 1'b1;             // 设置 ACK 错误标志
                    end
                end
            end

            DONE: begin
                done <= 1'b1;
                sda_oe <= 1'b0;
                if (!start) begin // 等待 start 信号清零后回到 IDLE
                    state <= IDLE;
                end
            end

            default: state <= IDLE;
        endcase
    end

endmodule
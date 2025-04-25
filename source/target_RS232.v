module target_RS232 (
    input wire clk,
    input wire rst_n,

    input wire RxD,
    output reg target_renew,
    output reg signed [23:0] target_height,
    output reg signed [23:0] target_pitch,
    output reg signed [23:0] target_roll,
    output reg signed [23:0] target_yaw
);

    // 串口模块
    wire RxD_idle;           // 串口空闲信号
    wire RxD_endofpacket;    // 数据包结束信号
    wire RxD_data_ready;     // 数据就绪信号
    wire [7:0] RxD_data;     // 接收到的串口数据
    reg [7:0] action_reg;    // 目标动作寄存器

    async_receiver #(
        .ClkFrequency(50000000),  // 50MHz
        .Baud(115200)
    ) inst_rs232 (
        .clk(clk),
        .RxD(RxD),
        .RxD_data_ready(RxD_data_ready),
        .RxD_data(RxD_data),
        .RxD_idle(RxD_idle),
        .RxD_endofpacket(RxD_endofpacket)
    );

    // 更新动作寄存器
    reg [2:0] cnt;  // <--- 修正：缺失了 cnt 寄存器定义
    reg [7:0] data_packed [2:0];
    localparam START_BYTE = 8'h0A;  // <--- 修正：你使用了 START_BYTE 和 STOP_BYTE 但没定义
    localparam STOP_BYTE  = 8'h08;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 0;
            data_packed[0] <= 0;
            data_packed[1] <= 0;
            data_packed[2] <= 0;
        end 
        else if (RxD_data_ready) begin
            data_packed[cnt] <= RxD_data;
            cnt <= (cnt == 3) ? 0 : cnt + 1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            action_reg <= 0;
        else if (cnt == 3) begin
            action_reg <= (data_packed[0] == START_BYTE) && (data_packed[2] == STOP_BYTE) ?
                          data_packed[1] : action_reg;  
        end
    end

    reg target_renew_pre;
    always @(posedge clk) begin
        target_renew_pre <= (cnt == 3) ? 1 : 0;
        target_renew <= target_renew_pre;
    end
        

    // 目标值映射
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_height <= 0;
            target_pitch <= 0;
            target_roll <= 0;
            target_yaw <= 0;
        end else begin
            case (action_reg)
                8'h01: begin  // 起飞
                    target_height <= 16'sd32767;  // 一直往上飞（修正原来写成65536了，16位最大值是65535）
                    target_pitch <= 0;
                    target_roll <= 0;
                    target_yaw <= 0;
                end
                8'h02: begin  // 悬停
                    target_height <= 16'sd32767;  // 保持当前高度
                    target_pitch <= 0;
                    target_roll <= 0;
                    target_yaw <= 0;
                end
                8'h03: begin  // 下降
                    target_height <= 0;
                    target_pitch <= 0;  
                    target_roll <= 0;
                    target_yaw <= 0;
                end
                8'h04: begin  // 前进
                    target_height <= 16'sd32767;
                    target_pitch <= 16'd500;  // 目标俯仰角+5°
                    target_roll <= 0;
                    target_yaw <= 0;
                end
                8'h05: begin  // 后退
                    target_height <= 16'sd32767;
                    target_pitch <= -16'sd500;  // 加 s 表示有符号负数
                    target_roll <= 0;
                    target_yaw <= 0;
                end
                8'h06: begin  // 向左
                    target_height <= 16'sd32767;
                    target_pitch <= 0;
                    target_roll <= 16'd500;
                    target_yaw <= 0;
                end
                8'h07: begin  // 向右
                    target_height <= 16'sd32767;
                    target_pitch <= 0;
                    target_roll <= -16'sd500;
                    target_yaw <= 0;
                end
                8'h08: begin  // 左旋转
                    target_height <= 16'sd32767;
                    target_pitch <= 0;
                    target_roll <= 0;
                    target_yaw <= 16'd500;
                end
                8'h09: begin  // 右旋转
                    target_height <= 16'sd32767;
                    target_pitch <= 0;
                    target_roll <= 0;
                    target_yaw <= -16'sd500;
                end
                default: begin
                    target_height <= 0;
                    target_pitch <= 0;
                    target_roll <= 0;
                    target_yaw <= 0;
                end
            endcase
        end
    end

endmodule

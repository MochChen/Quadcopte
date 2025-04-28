module register_map (
    input  wire        clk,
    input  wire        rst_n,

    // axi接口
    input  wire [31:0] axi_din_32,    // axi接收到32bit
    output wire [31:0] axi_dout_32,   // 发送到axi的32bit

    // 输出给其他模块使用的参数
    output reg  signed [15:0] pwm_base,         // 悬停时的基础PWM值
    output reg  signed [15:0] tilt_cmp,         // 倾斜时的升力补偿
    output reg  signed [7:0] use_INT,           // 是否启用INT作位采集频率
    output reg  signed [15:0] delay_cnt,        // 延迟计数
    input  reg  [7:0] error_code,              // 错误代码
    input  reg  signed [15:0] mpu_temperature,  // MPU温度
    output reg  [7:0] mpu_device_addr,         // MPU设备地址
    output reg  [7:0] mpu_register_addr,       // MPU寄存器地址
    output reg  [7:0] mpu_register_data,       // MPU寄存器数据
    output reg  [7:0] mpu_read_start_addr,     // MPU读取起始地址
    output reg  [7:0] iterations,              // 迭代次数
    output reg  [23:0] tgt_height,             // 目标高度
    output reg  [23:0] tgt_pitch,              // 目标pitch
    output reg  [23:0] tgt_roll,               // 目标roll
    output reg  [23:0] tgt_yaw,                // 目标yaw
    output reg  [23:0] action_reg,             // 目标动作
    output reg  [7:0] alpha,                   // 互补滤波系数
    output reg  [15:0] max_speed,              // pwm输出最大占空比
    output reg  [15:0] min_speed,              // pwm输出最小占空比
    output reg  [15:0] KP,                     // KP比例系数
    output reg  [15:0] KI,                     // KI积分
    output reg  [15:0] KD                      // KD微分
);

    // 提取地址和值的高低位
    wire [7:0] address = axi_din_32[31:24];  // 地址字段（高 8 位）
    wire [7:0] register_8bit = axi_din_32[7:0];   // 低 8 位寄存器值
    wire [15:0] register_16bit = axi_din_32[15:0]; // 低 16 位寄存器值
    wire [23:0] register_24bit = axi_din_32[23:0]; // 低 24 位寄存器值

报警输出的数据由ps端口读取，用延迟即可。
收到的数据定期更新，使用
    // 写寄存器逻辑
    always @(posedge clk or posedge rst_n) begin
        if (!rst_n) begin
            // 上电复位时给默认值
            pwm_base <= 16'd30000;  // 默认悬停时的基础PWM值
            tilt_cmp <= 16'd5000;   // 默认倾斜时的升力补偿
            use_INT <= 8'd0;        // 默认是否启用INT作位采集频率
            delay_cnt <= 16'd49999; // 默认MPU初始化完成到第一次开始采集的延迟时间
            mpu_device_addr <= 8'h68; // 默认MPU设备地址
            mpu_register_addr <= 8'h6b; // 默认MPU寄存器地址
            mpu_register_data <= 8'h00; // 默认MPU寄存器数据
            mpu_read_start_addr <= 8'h3b; // 默认MPU读取起始地址
            iterations <= 8'd12;    // 默认CORDIC迭代次数
            tgt_height <= 24'd0;    // 默认目标高度
            tgt_pitch <= 24'd0;     // 默认目标pitch
            tgt_roll <= 24'd0;      // 默认目标roll
            tgt_yaw <= 24'd0;       // 默认目标yaw
            action_reg <= 24'd0;    // 默认目标动作
            alpha <= 8'd99;         // 默认互补滤波系数
            max_speed <= 16'd65535; // 默认最大占空比
            min_speed <= 16'd256;   // 默认最小占空比
            KP <= 16'd0;            // 默认KP比例系数
            KI <= 16'd0;            // 默认KI积分
            KD <= 16'd0;            // 默认KD微分
        end else begin
            if (rx_valid) begin
                case (address)
                    8'd0: pwm_base <= register_16bit;                 // 地址0写入 pwm_base
                    8'd1: tilt_cmp <= register_16bit;                 // 地址1写入 tilt_cmp
                    8'd2: use_INT <= register_8bit;                   // 地址2写入 use_INT
                    8'd3: delay_cnt <= register_16bit;                // 地址3写入 delay_cnt
                    8'd6: mpu_device_addr <= register_8bit;           // 地址6写入 mpu_device_addr
                    8'd7: mpu_register_addr <= register_8bit;         // 地址7写入 mpu_register_addr
                    8'd8: mpu_register_data <= register_8bit;         // 地址8写入 mpu_register_data
                    8'd9: mpu_read_start_addr <= register_8bit;       // 地址9写入 mpu_read_start_addr
                    8'd10: iterations <= register_8bit;               // 地址10写入 iterations
                    8'd11: tgt_height <= register_24bit;              // 地址11写入 tgt_height
                    8'd12: tgt_pitch <= register_24bit;               // 地址12写入 tgt_pitch
                    8'd13: tgt_roll <= register_24bit;                // 地址13写入 tgt_roll
                    8'd14: tgt_yaw <= register_24bit;                 // 地址14写入 tgt_yaw
                    8'd15: action_reg <= register_24bit;              // 地址15写入 action_reg
                    8'd16: alpha <= register_8bit;                    // 地址16写入 alpha
                    8'd17: max_speed <= register_16bit;               // 地址17写入 max_speed
                    8'd18: min_speed <= register_16bit;               // 地址18写入 min_speed
                    8'd19: KP <= register_16bit;                      // 地址19写入 KP
                    8'd20: KI <= register_16bit;                      // 地址20写入 KI
                    8'd21: KD <= register_16bit;                      // 地址21写入 KD
                    default: ; // 地址不匹配时什么也不做
                endcase
            end
        end
    end

endmodule

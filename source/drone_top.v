`include "bb_mpu.sv"
`include "cordic_angle.v"
`include "bb_pwm.sv"
`include "bb_pid.v"
`include "async.v"

module drone_top (
    input clk,              // 50MHz主时钟
    input reset,            // 高有效复位

    //output out_of_control,   // 失控信号
    output scl,        // I2C时钟
    inout sda,         // I2C数据
    output pwm_1_out,    // PWM输出
    output pwm_2_out, 
    output pwm_3_out, 
    output pwm_4_out, 

    input RxD
);

    // rs232模块信号
    wire RxD_idle;
    wire RxD_endofpacket;
    wire RxD_data_ready;
    wire [7:0] RxD_data;
    reg [7:0] action_reg;
    wire is_move = (action_reg = 8'h3);
    reg [15:0] target_height; // 目标高度
    reg [15:0] target_pitch;  // 目标俯仰角
    reg [15:0] target_roll;   // 目标滚转角
    reg [15:0] target_yaw;    // 目标偏航角

    async_receiver #(
        .ClkFrequency(50000000),	// 50MHz
	    .Baud(115200)
    ) inst_rs232 (
        .clk(clk),
        .RxD(RxD),
        .RxD_data_ready(RxD_data_ready),
        .RxD_data(RxD_data),
        .RxD_idle(RxD_idle),  // asserted when no data has been received for a while
        .RxD_endofpacket(RxD_endofpacket)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!reset) 
            action_reg <= 8'h0;
        else if (RxD_data_ready) 
            action_reg <= RxD_data;
    end

    always @(posedge clk or negedge rst_n) begin
    if (!reset) begin
        target_height <= 0;
        target_pitch <= 0;
        target_roll <= 0;
        target_yaw <= 0;
    end else begin
        case (action_reg) // 目标动作寄存器
            8'h01: begin // 起飞
                target_height <= 16'd1000; // 1米，单位可以自定义
                target_pitch <= 0;
                target_roll <= 0;
                target_yaw <= 0;
            end
            8'h02: begin // 悬停
                target_height <= current_height; // 保持当前高度
                target_pitch <= 0;
                target_roll <= 0;
                target_yaw <= 0;
            end
            8'h03: begin // 前进
                target_height <= current_height;
                target_pitch <= 16'd500; // 假设+5°，单位自定义
                target_roll <= 0;
                target_yaw <= 0;
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

    // MPU模块信号
    wire rst_n = ~reset;    // 转换为低有效
    reg mpu_init;
    wire mpu_init_done;
    reg mpu_transfer;
    wire mpu_data_oe;
    wire [7:0] mpu_data;
    wire mpu_busy;

    bb_mpu #(
        .CLK_MAIN(50000000),
        .SCL_DIV(800000)
    ) inst_mpu (
        .clk(clk),
        .scl(scl),
        .sda(sda),
        .rst_n(rst_n),
        .mpu_init(mpu_init),
        .init_done(mpu_init_done),
        .mpu_transfer(mpu_transfer),
        .data_avalid(mpu_data_oe),
        .data(mpu_data),
        .busy_now(mpu_busy)
    );

    // cordic module
    cordic_angle #(
        .ITERATIONS(12)
    ) inst_angle (
        .clk(clk),
        .rst_n(reset),
        .x_in(ax),
        .y_in(ay),
        .z_in(az),
        .start(angle_start),
        .done(angle_done),
        .pitch(pitch_accel),
        .roll(roll_accel)
    )

    // PWM模块信号
    reg [15:0] PWM_M1;
    reg m1_oe;
    wire m1_busy;
    // PWM模块实例化（单电机示例）
    bb_pwm #(
        .MAX_SPEED(65536),
        .MIN_SPEED(256),
        .ACC(2560),
        .DEAD_ZONE(1280),
        .STATE_WIDTH(3)
    ) inst_pwm_m1 (
        .clk(clk),
        .rst(reset),
        .speed_in(PWM_M1),
        .speed_oe(m1_oe),
        .pwm_out(pwm_1_out),
        .busy(m1_busy)
    );



    // 三段式状态机
    typedef enum reg [2:0] {IDLE, MPU_CAPTURE, CONTROL, PWM_OUT, ERROR} state_t;
    state_t state, next_state;

    // 时序逻辑
    always @(posedge clk or posedge reset) begin
        if (reset) state <= IDLE;
        else state <= next_state;
    end

    // 组合逻辑
    always @(*) begin
        case (state)
            IDLE: next_state = mpu_init_done ? MPU_CAPTURE : IDLE;//在这里初始化
            MPU_CAPTURE: next_state = capture_done ? CURRENT : MPU_CAPTURE;//获取12byte的数据
            CURRENT: next_state = posture_is_confirmed ? TARGET : CURRENT;//计算当前角度
            TARGET: next_state = PID_CONTROL;//查看目标角度
            PID_CONTROL: next_state = PWM_OUT;//计算输出
            PWM_OUT: next_state = MPU_CAPTURE;//输出
            ERROR:;
            default: next_state = IDLE;
        endcase
    end



    // 输出
    reg [7:0] mpu_data_packed [0:11];
    reg [3:0] byte_counter;
    
    wire signed [31:0] ax = {mpu_data_packed[0:1]} * 10000 / 16384;  // 1g = 10000
    wire signed [31:0] ay = {mpu_data_packed[2:3]} * 10000 / 16384;  // ax, ay, az 是物理加速度值（单位 g 或 m/s²）
    wire signed [31:0] az = {mpu_data_packed[4:5]} * 10000 / 16384;
    wire signed [31:0] gyro_x = {mpu_data_packed[6:7]} * 10000 / 16384;  // 1弧度 = 10000
    wire signed [31:0] gyro_y = {mpu_data_packed[8:9]} * 10000 / 16384;
    wire signed [31:0] gyro_z = {mpu_data_packed[10:11]} * 10000 / 16384;

    parameter signed [15:0] ALPHA = 98; // 0.98 (放大100倍避免小数计算)

    reg [15:0] prev_error_pitch;  // 上一次的俯仰角误差
    reg [15:0] prev_error_roll;   // 上一次的滚转角误差
    reg [15:0] prev_error_yaw;    // 上一次的偏航角误差

    // 微分用滤波器时
    // reg [15:0] filtered_derivative_pitch;
    // always @(posedge clk) begin
    //     filtered_derivative_pitch <= (Derivative_Pitch_error + filtered_derivative_pitch) >> 1; // 简单平均滤波
    // end

    always @(posedge clk or posedge reset) begin
        if (!rst_n) begin
            mpu_data_packed [0] <= 0;
            mpu_data_packed [1] <= 0;
            mpu_data_packed [2] <= 0;
            mpu_data_packed [3] <= 0;
            mpu_data_packed [4] <= 0;
            mpu_data_packed [5] <= 0;
            mpu_data_packed [6] <= 0;
            mpu_data_packed [7] <= 0;
            mpu_data_packed [8] <= 0;
            mpu_data_packed [9] <= 0;
            mpu_data_packed [10] <= 0;
            mpu_data_packed [11] <= 0;
            byte_counter <= 0;
            capture_done <= 0;

            prev_error_pitch <= 0;
            prev_error_roll <= 0;
            prev_error_yaw <= 0;
            Integral_Pitch_error <= 0;
            Derivative_Pitch_error <= 0;
        end
        else case (state)
            MPU_CAPTURE: begin
                if (mpu_data_oe) begin
                    if (byte_counter == 12) begin
                        byte_counter <= 0; // Reset byte_counter
                        capture_done <= 1;
                        angle_start <= 1;
                        // 角度积分
                        pitch_gyro <= pitch_gyro + gyro_x * dt;
                        roll_gyro  <= roll_gyro + gyro_y * dt;
                        yaw_gyro   <= yaw_gyro + gyro_z * dt; //current_yaw == yaw_gyro
                    end
                    else begin
                        byte_counter <= byte_counter + 1; // Increment byte_counter
                        mpu_data_packed[byte_counter] <= mpu_data; // Store data
                    end
                end
            end
            CURRENT: begin
                capture_done <= 0;
                angle_start <= 0;
                if (angle_done) begin
                    current_pitch <= (ALPHA * pitch_gyro + (100 - ALPHA) * pitch_accel) / 100;//互补滤波
                    current_roll  <= (ALPHA * roll_gyro + (100 - ALPHA) * roll_accel) / 100;
                    posture_is_confirmed <= 1;
                end
            end
            TARGET: begin
                // 计算当前误差
                error_height <= target_height - current_height;
                error_pitch <= target_pitch - current_pitch;
                error_roll <= target_roll - current_roll;
                error_yaw <= target_yaw - yaw_gyro;
                PWM_base <= is_move ? PWM_BASE + TILT_CMP : PWM_BASE;

                // 积分项、微分项
                Integral_Pitch_error <= Integral_Pitch_error + error_pitch;
                Integral_Roll_error <= Integral_Roll_error + error_roll;
                Integral_Yaw_error <= Integral_Yaw_error + error_yaw;

                // 微分项：当前误差 - 上一次误差
                Derivative_Pitch_error <= error_pitch - prev_error_pitch;
                Derivative_Roll_error <= error_roll - prev_error_roll;
                Derivative_Yaw_error <= error_yaw - prev_error_yaw;

                // 更新前一次误差
                prev_error_pitch <= error_pitch;  // 保存当前误差为下一次使用
                prev_error_roll <= error_roll;
                prev_error_yaw <= error_yaw;
            end 
            PID_CONTROL:begin
                m1_oe <= 1;
                PWM_M1 = PWM_base - (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error) 
                                - (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error) 
                                - (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error);

                PWM_M2 = PWM_base - (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                                + (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error)
                                + (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error);

                PWM_M3 = PWM_base + (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                                - (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error)
                                + (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error);

                PWM_M4 = PWM_base + (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                                + (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error)
                                - (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error);
            end
            PWM_OUT:begin
                m1_oe <= 0;
                
            end
        endcase 
        end
    end

endmodule
    // PWM_base = pwm_悬停 + （上下 or 前后左右补偿）：在移动时，倾斜会损失部分垂直升力，可能导致高度下降。
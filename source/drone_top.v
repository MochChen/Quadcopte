 `include "mpu_top.v"
 `include "mpu_mid.v"
 `include "std_iic_master.v"
 `include "cordic_angle.v"
 `include "pwm.v"
 `include "async_receiver.v"


module drone_top #(
    parameter PWM_BASE = 16'd30000,  // 悬停时的基础PWM值
    parameter TILT_CMP = 16'd5000   // 倾斜时的升力补偿
)(
    input clk,              // 50MHz主时钟
    input rst_n,            // 高有效复位
    output scl,             // I2C时钟
    inout sda,              // I2C数据
    input signal_INT,
    output pwm_1_out,       // PWM输出至电机1
    output pwm_2_out,       // PWM输出至电机2
    output pwm_3_out,       // PWM输出至电机3
    output pwm_4_out,       // PWM输出至电机4
    input RxD               // 串口接收数据
);

    // MPU模块
    reg mpu_init_start;
    reg mpu_read_start;
    wire mpu_init_done;
    wire mpu_read_done;
    wire mpu_data_avalid;
    wire [7:0] mpu_data;

    mpu_top mpu_ins (
        .clk(clk),
        .rst_n(rst_n),
        .init_start(mpu_init_start),
        .read_start(mpu_read_start),
        .init_done(mpu_init_done),
        .read_done(mpu_read_done),
        .data_avalid(mpu_data_avalid),
        .data(mpu_data),
        .scl(scl),
        .sda(sda)
    );

    // CORDIC角度计算模块
    wire signed [15:0] ax = $signed({mpu_data_packed[0], mpu_data_packed[1]});
    wire signed [15:0] ay = $signed({mpu_data_packed[2], mpu_data_packed[3]});
    wire signed [15:0] az = $signed({mpu_data_packed[4], mpu_data_packed[5]});
    reg pitch_start;
    wire pitch_done;
    wire [15:0] pitch_acc;
    reg [15:0] cur_pitch_acc;
    cordic_angle pitch_inst (
        .clk(clk),
        .rst_n(rst_n),
        .x(ax),
        .y(ay),
        .z(az),
        .cdra_start(pitch_start),
        .cdra_done(pitch_done),
        .crda_angle(pitch_acc)
    );

    reg roll_start;
    wire roll_done;
    wire [15:0] roll_acc;
    reg [15:0] cur_roll_acc;
    cordic_angle roll_inst (
        .clk(clk),
        .rst_n(rst_n),
        .x(ay), //更换位置就是roll
        .y(ax),
        .z(az),
        .cdra_start(roll_start),
        .cdra_done(roll_done),
        .crda_angle(roll_acc)
    );


    parameter   IDLE         = 3'h0,
                INIT_MPU     = 3'h1,
                MPU_CAPTURE  = 3'h1,
                CRD_PITCH    = 3'h2,
                CRD_ROLL     = 3'h3,
                PID_CONTROL  = 3'h4,
                PWM_OUT      = 3'h5,
                ERROR        = 3'h6;
    reg [2:0] state = IDLE;
    reg mpu_init_done_reg;
    reg [7:0] mpu_data_packed [13:0]; //一共14字节
    reg [3:0] byte_cnt = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            case (state)
                IDLE:
                    begin
                        if (!mpu_init_done_reg) begin
                            state <= INIT_MPU;
                            mpu_init_start <= 1;
                        end else begin
                            if (signal_INT) begin
                                state <= MPU_CAPTURE;
                                mpu_read_start <= 1;
                            end else begin
                                state <= IDLE;
                            end
                        end
                    end
                INIT_MPU:
                    begin
                        mpu_init_start <= 0;
                        if (mpu_init_done) begin
                            mpu_init_done_reg <= 1;
                            state <= IDLE;
                        end
                    end
                MPU_CAPTURE:
                    begin
                        mpu_read_start <= 0;
                        if (mpu_read_done) begin
                            state <= CRD_PITCH;
                            byte_cnt <= 0;
                            pitch_start <= 1;
                        end
                        // 在这里读取14字节数据
                        if (mpu_data_avalid) begin
                            mpu_data_packed[byte_cnt] <= mpu_data;
                            byte_cnt <= byte_cnt + 1;
                        end
                    end
                CRD_PITCH:
                    begin
                        pitch_start <= 0;
                        if (pitch_done) begin
                            cur_pitch_acc <= pitch_acc;
                            state <= CRD_ROLL;
                            roll_start <= 1;
                        end
                    end
                CRD_ROLL:
                    begin
                        roll_start <= 0;
                        if (roll_done) begin
                            cur_roll_acc <= roll_acc;
                            state <= CMP_FILTER;
                        end
                    end
                CMP_FILTER:
                    begin
                        current_pitch <= (ALPHA * pitch_gyro + (100 - ALPHA) * cur_pitch_acc) / 100;
                        current_roll  <= (ALPHA * roll_gyro  + (100 - ALPHA) * cur_roll_acc) / 100;
                    end

                default: state <= IDLE;
            endcase
        end
    end
    
    
endmodule

// // MPU数据采集和姿态计算
//     reg [7:0] mpu_data_packed [0:11];  // MPU原始数据包
//     reg [3:0] byte_counter;            // 数据字节计数器
//     reg capture_done;                  // 数据采集完成标志
//     reg posture_is_confirmed;          // 姿态计算完成标志
//     reg [15:0] pitch_gyro, roll_gyro, yaw_gyro;  // 陀螺仪积分角度
//     reg [15:0] current_pitch, current_roll;      // 当前姿态
//     parameter signed [15:0] ALPHA = 98;          // 互补滤波系数 (0.98 * 100)
//     parameter signed dt = 16'd1;                        // 假设时间步长（需根据实际采样率调整）
//     integer i;

//     // 加速度和陀螺仪数据转换
//     wire signed [15:0] raw_ax = {mpu_data_packed[0], mpu_data_packed[1]};
//     wire signed [15:0] raw_ay = {mpu_data_packed[2], mpu_data_packed[3]};
//     wire signed [15:0] raw_az = {mpu_data_packed[4], mpu_data_packed[5]};
//     wire signed [15:0] raw_gyro_x = {mpu_data_packed[6], mpu_data_packed[7]};
//     wire signed [15:0] raw_gyro_y = {mpu_data_packed[8], mpu_data_packed[9]};
//     wire signed [15:0] raw_gyro_z = {mpu_data_packed[10], mpu_data_packed[11]};

//     assign ax = raw_ax * 10000 / 16384;
//     assign ay = raw_ay * 10000 / 16384;
//     assign az = raw_az * 10000 / 16384;
//     assign gyro_x = raw_gyro_x * 10000 / 131;   // 陀螺仪分辨率通常是 131 LSB/(°/s)
//     assign gyro_y = raw_gyro_y * 10000 / 131;
//     assign gyro_z = raw_gyro_z * 10000 / 131;



//     // PID相关信号
//     reg [15:0] PWM_base;              // 基础PWM值
//     reg [15:0] error_height;          // 高度误差
//     reg [15:0] error_pitch, error_roll, error_yaw;  // 姿态误差
//     reg [15:0] Integral_Pitch_error, Integral_Roll_error, Integral_Yaw_error;  // 积分项
//     reg [15:0] Derivative_Pitch_error, Derivative_Roll_error, Derivative_Yaw_error;  // 微分项
//     reg [15:0] prev_error_pitch, prev_error_roll, prev_error_yaw;  // 上一次误差
//     parameter Kp_pitch = 16'd100, Ki_pitch = 16'd10, Kd_pitch = 16'd50;  // PID参数（示例值）
//     parameter Kp_roll = 16'd100, Ki_roll = 16'd10, Kd_roll = 16'd50;
//     parameter Kp_yaw = 16'd80, Ki_yaw = 16'd5, Kd_yaw = 16'd30;

//     // 状态机时序逻辑
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) state <= IDLE;
//         else state <= next_state;
//     end

//     // 状态机组合逻辑
//     always @(*) begin
//         case (state)
//             IDLE: next_state = mpu_init_done ? MPU_CAPTURE : IDLE;
//             MPU_CAPTURE: next_state = capture_done ? CURRENT : MPU_CAPTURE;
//             CURRENT: next_state = posture_is_confirmed ? TARGET : CURRENT;
//             TARGET: next_state = PID_CONTROL;
//             PID_CONTROL: next_state = PWM_OUT;
//             PWM_OUT: next_state = MPU_CAPTURE;
//             ERROR: next_state = IDLE;  // 错误状态返回IDLE
//             default: next_state = IDLE;
//         endcase
//     end

//     // 主状态机输出逻辑
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             mpu_init <= 1;
//             for (i = 0; i < 12; i = i + 1) mpu_data_packed[i] <= 8'd0;
//             mpu_transfer <= 0;
//             byte_counter <= 0;
//             capture_done <= 0;
//             posture_is_confirmed <= 0;
//             pitch_gyro <= 0;
//             roll_gyro <= 0;
//             yaw_gyro <= 0;
//             prev_error_pitch <= 0;
//             prev_error_roll <= 0;
//             prev_error_yaw <= 0;
//             Integral_Pitch_error <= 0;
//             Integral_Roll_error <= 0;
//             Integral_Yaw_error <= 0;
//             Derivative_Pitch_error <= 0;
//             Derivative_Roll_error <= 0;
//             Derivative_Yaw_error <= 0;
//             pwm_M1 <= 0;
//             pwm_M2 <= 0;
//             pwm_M3 <= 0;
//             pwm_M4 <= 0;
//             m1_oe <= 0;
//             m2_oe <= 0;
//             m3_oe <= 0;
//             m4_oe <= 0;
//         end else begin
//             case (state)
//                 IDLE: begin
//                     mpu_init <= 0;  // 初始化完成时关闭初始化信号
//                 end
//                 MPU_CAPTURE: begin
//                     mpu_transfer <= 1;
//                     if (mpu_data_oe) begin
//                         if (byte_counter == 11) begin
//                             byte_counter <= 0;
//                             capture_done <= 1;
//                             angle_start <= 1;
//                             //mpu_transfer <= 0;
//                             // 陀螺仪角度积分
//                             pitch_gyro <= pitch_gyro + gyro_x * dt;
//                             roll_gyro <= roll_gyro + gyro_y * dt;
//                             yaw_gyro <= yaw_gyro + gyro_z * dt;
//                         end else begin
//                             byte_counter <= byte_counter + 1;
//                             mpu_data_packed[byte_counter] <= mpu_data;
//                         end
//                     end
//                 end
//                 CURRENT: begin
//                     capture_done <= 0;
//                     angle_start <= 0;
//                     if (angle_done) begin
//                         // 互补滤波计算当前姿态
//                         current_pitch <= (ALPHA * pitch_gyro + (100 - ALPHA) * pitch_accel) / 100;
//                         current_roll <= (ALPHA * roll_gyro + (100 - ALPHA) * roll_accel) / 100;
//                         posture_is_confirmed <= 1;
//                     end
//                 end
//                 TARGET: begin
//                     posture_is_confirmed <= 0;
//                     // 计算误差
//                     error_height <= target_height - current_height;
//                     error_pitch <= target_pitch - current_pitch;
//                     error_roll <= target_roll - current_roll;
//                     error_yaw <= target_yaw - yaw_gyro;
//                     PWM_base <= is_move ? PWM_BASE + TILT_CMP : PWM_BASE;

//                     // 更新PID积分项和微分项
//                     Integral_Pitch_error <= Integral_Pitch_error + error_pitch;//可能会溢出:
//                     Integral_Roll_error <= Integral_Roll_error + error_roll;
//                     Integral_Yaw_error <= Integral_Yaw_error + error_yaw;
//                     Derivative_Pitch_error <= (error_pitch - prev_error_pitch) * 90 / 100;
//                     Derivative_Roll_error <= (error_roll - prev_error_roll) * 90 / 100;
//                     Derivative_Yaw_error <= (error_yaw - prev_error_yaw) * 90 / 100;

//                     // 保存当前误差为下一次使用
//                     prev_error_pitch <= error_pitch;
//                     prev_error_roll <= error_roll;
//                     prev_error_yaw <= error_yaw;
//                 end
//                 PID_CONTROL: begin
//                     m1_oe <= 1;
//                     m2_oe <= 1;
//                     m3_oe <= 1;
//                     m4_oe <= 1;
//                     // 计算四个电机的PWM值
//                     pwm_M1 <= PWM_base - (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
//                                        - (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
//                                        - (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
//                     pwm_M2 <= PWM_base - (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
//                                        + (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
//                                        + (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
//                     pwm_M3 <= PWM_base + (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
//                                        - (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
//                                        + (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
//                     pwm_M4 <= PWM_base + (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
//                                        + (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
//                                        - (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
//                 end
//                 PWM_OUT: begin
//                     m1_oe <= 0;
//                     m2_oe <= 0;
//                     m3_oe <= 0;
//                     m4_oe <= 0;
//                 end
//             endcase
//         end
//     end

//     // PWM模块
//     reg [15:0] pwm_M1, pwm_M2, pwm_M3, pwm_M4;  // 四个电机的PWM值
//     reg m1_oe, m2_oe, m3_oe, m4_oe;             // PWM输出使能
//     wire m1_busy, m2_busy, m3_busy, m4_busy;    // PWM忙碌信号

//     pwm #(
//         .MAX_SPEED(65536),
//         .MIN_SPEED(256),
//         .ACC(2560),
//         .DEAD_ZONE(1280),
//         .STATE_WIDTH(3)
//     ) inst_pwm_M1 (
//         .clk(clk),
//         .rst_n(rst_n),
//         .speed_in(pwm_M1),
//         .speed_oe(m1_oe),
//         .pwm_out(pwm_1_out),
//         .busy(m1_busy)
//     );

//     pwm #(
//         .MAX_SPEED(65536),
//         .MIN_SPEED(256),
//         .ACC(2560),
//         .DEAD_ZONE(1280),
//         .STATE_WIDTH(3)
//     ) inst_pwm_M2 (
//         .clk(clk),
//         .rst_n(rst_n),
//         .speed_in(pwm_M2),
//         .speed_oe(m2_oe),
//         .pwm_out(pwm_2_out),
//         .busy(m2_busy)
//     );

//     pwm #(
//         .MAX_SPEED(65536),
//         .MIN_SPEED(256),
//         .ACC(2560),
//         .DEAD_ZONE(1280),
//         .STATE_WIDTH(3)
//     ) inst_pwm_M3 (
//         .clk(clk),
//         .rst_n(rst_n),
//         .speed_in(pwm_M3),
//         .speed_oe(m3_oe),
//         .pwm_out(pwm_3_out),
//         .busy(m3_busy)
//     );

//     pwm #(
//         .MAX_SPEED(65536),
//         .MIN_SPEED(256),
//         .ACC(2560),
//         .DEAD_ZONE(1280),
//         .STATE_WIDTH(3)
//     ) inst_pwm_M4 (
//         .clk(clk),
//         .rst_n(rst_n),
//         .speed_in(pwm_M4),
//         .speed_oe(m4_oe),
//         .pwm_out(pwm_4_out),
//         .busy(m4_busy)
//     );

//     // 串口模块
//     wire RxD_idle;           // 串口空闲信号
//     wire RxD_endofpacket;    // 数据包结束信号
//     wire RxD_data_ready;     // 数据就绪信号
//     wire [7:0] RxD_data;     // 接收到的串口数据
//     reg [7:0] action_reg;    // 目标动作寄存器
//     wire is_move = (action_reg == 8'h03);  // 是否为前进状态
//     reg [15:0] target_height;  // 目标高度
//     reg [15:0] target_pitch;   // 目标俯仰角
//     reg [15:0] target_roll;    // 目标滚转角
//     reg [15:0] target_yaw;     // 目标偏航角
//     reg [15:0] current_height;  // 当前高度（假设由外部输入或计算）

//     async_receiver #(
//         .ClkFrequency(50000000),  // 50MHz
//         .Baud(115200)
//     ) inst_rs232 (
//         .clk(clk),
//         .RxD(RxD),
//         .RxD_data_ready(RxD_data_ready),
//         .RxD_data(RxD_data),
//         .RxD_idle(RxD_idle),
//         .RxD_endofpacket(RxD_endofpacket)
//     );

//     // 更新动作寄存器
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) action_reg <= 8'h00; 
//         else if (RxD_data_ready) action_reg <= RxD_data;
//     end

//     // 目标值映射
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             target_height <= 0;
//             target_pitch <= 0;
//             target_roll <= 0;
//             target_yaw <= 0;
//         end else begin
//             case (action_reg)
//                 8'h01: begin  // 起飞
//                     target_height <= 16'd1000;  // 目标高度1米
//                     target_pitch <= 0;
//                     target_roll <= 0;
//                     target_yaw <= 0;
//                 end
//                 8'h02: begin  // 悬停
//                     target_height <= current_height;  // 保持当前高度
//                     target_pitch <= 0;
//                     target_roll <= 0;
//                     target_yaw <= 0;
//                 end
//                 8'h03: begin  // 前进
//                     target_height <= current_height;
//                     target_pitch <= 16'd500;  // 目标俯仰角+5°
//                     target_roll <= 0;
//                     target_yaw <= 0;
//                 end
//                 default: begin
//                     target_height <= 0;
//                     target_pitch <= 0;
//                     target_roll <= 0;
//                     target_yaw <= 0;
//                 end
//             endcase
//         end
//     end

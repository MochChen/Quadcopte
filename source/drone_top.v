`include "bb_mpu.sv"
`include "bb_pwm.sv"
`include "bb_pid.v"
`include "async.v"

module drone_top (
    input clk,              // 50MHz主时钟
    input reset,            // 高有效复位
    output scl,             // I2C时钟
    inout sda,              // I2C数据
    output pwm_1_out,       // PWM输出至电机1
    output pwm_2_out,       // PWM输出至电机2
    output pwm_3_out,       // PWM输出至电机3
    output pwm_4_out,       // PWM输出至电机4
    input RxD               // 串口接收数据

    //output out_of_control,   // 失控信号
    output scl,        // I2C时钟
    inout sda,         // I2C数据
    output pwm_out,    // PWM输出（单电机示例）

    input RxD
);

    // 参数定义
    parameter PWM_BASE = 16'd30000;  // 悬停时的基础PWM值
    parameter TILT_CMP = 16'd5000;   // 倾斜时的升力补偿

    // 串口模块信号
    wire RxD_idle;           // 串口空闲信号
    wire RxD_endofpacket;    // 数据包结束信号
    wire RxD_data_ready;     // 数据就绪信号
    wire [7:0] RxD_data;     // 接收到的串口数据
    reg [7:0] action_reg;    // 目标动作寄存器
    wire is_move = (action_reg == 8'h03);  // 是否为前进状态
    reg [15:0] target_height;  // 目标高度
    reg [15:0] target_pitch;   // 目标俯仰角
    reg [15:0] target_roll;    // 目标滚转角
    reg [15:0] target_yaw;     // 目标偏航角
// rs232模块信号 /////////////////////////////////////////////////////////////////
    wire RxD_idle;
    wire RxD_endofpacket;
    wire RxD_data_ready;
    wire [7:0] RxD_data;

    // 串口模块实例化
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
    always @(posedge clk or posedge reset) begin
        if (reset) 
            action_reg <= 8'h00;
        else if (RxD_data_ready) 
            action_reg <= RxD_data;
    end

    // 目标值映射
    reg [15:0] current_height;  // 当前高度（假设由外部输入或计算）
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            target_height <= 0;
            target_pitch <= 0;
            target_roll <= 0;
            target_yaw <= 0;
        end else begin
            case (action_reg)
                8'h01: begin  // 起飞
                    target_height <= 16'd1000;  // 目标高度1米
                    target_pitch <= 0;
                    target_roll <= 0;
                    target_yaw <= 0;
                end
                8'h02: begin  // 悬停
                    target_height <= current_height;  // 保持当前高度
                    target_pitch <= 0;
                    target_roll <= 0;
                    target_yaw <= 0;
                end
                8'h03: begin  // 前进
                    target_height <= current_height;
                    target_pitch <= 16'd500;  // 目标俯仰角+5°
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
    wire rst_n = ~reset;    // 低有效复位
    reg mpu_init;           // MPU初始化信号
    wire mpu_init_done;     // MPU初始化完成
    reg mpu_transfer;       // MPU数据传输信号
    wire mpu_data_oe;       // MPU数据有效
    wire [7:0] mpu_data;    // MPU数据
    wire mpu_busy;          // MPU忙碌信号

    // MPU模块实例化

// MPU模块信号  /////////////////////////////////////////////////////////////////
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

    // CORDIC角度计算模块信号
    reg angle_start;        // 角度计算开始信号
    wire angle_done;        // 角度计算完成信号
    wire [15:0] pitch_accel;// 加速度计计算的俯仰角
    wire [15:0] roll_accel; // 加速度计计算的滚转角
    wire signed [31:0] ax, ay, az;  // 加速度数据
    wire signed [31:0] gyro_x, gyro_y, gyro_z;  // 陀螺仪数据

    // CORDIC模块实例化
    cordic_angle #(
        .ITERATIONS(12)
    ) inst_angle (
        .clk(clk),
        .rst_n(rst_n),  // 修正为低有效复位
        .x_in(ax),
        .y_in(ay),
        .z_in(az),
        .start(angle_start),
        .done(angle_done),
        .pitch(pitch_accel),
        .roll(roll_accel)
    );

    // PWM模块信号
    reg [15:0] PWM_M1, PWM_M2, PWM_M3, PWM_M4;  // 四个电机的PWM值
    reg m1_oe, m2_oe, m3_oe, m4_oe;             // PWM输出使能
    wire m1_busy, m2_busy, m3_busy, m4_busy;    // PWM忙碌信号

    // PWM模块实例化（四个电机）
// PWM模块信号 /////////////////////////////////////////////////////////////////
    reg [15:0] pid_output;
    reg pwm_update;
    wire pwm_busy;
    // PWM模块实例化（单电机示例）
    bb_pwm #(
        .MAX_SPEED(65536),
        .MIN_SPEED(256),
        .ACC(2560),
        .DEAD_ZONE(1280),
        .STATE_WIDTH(3)
    ) inst_pwm (
        .clk(clk),
        .rst(reset),
        .speed_in(pid_output),
        .speed_oe(pwm_update),
        .pwm_out(pwm_out),
        .busy(pwm_busy)
    );

// PID模块信号 /////////////////////////////////////////////////////////////////
    reg calc_pid_oe;
    reg [15:0] target_from_232, current_from_mpu;
    wire to_pwm_oe;
    wire [15:0] to_pwm;

    bb_pid #(
        .KP (16'd10),
        .KI (16'd2),
        .KD (16'd5)
    ) inst_pid (
        .clk(clk),
        .rst(reset),
        .calc_pid_oe(calc_pid_oe),
        .target_from_232(target_from_232),
        .current_from_mpu(current_from_mpu),
        .to_pwm_oe(to_pwm_oe),
        .to_pwm(to_pwm)
    );

    bb_pwm #(
        .MAX_SPEED(65536),
        .MIN_SPEED(256),
        .ACC(2560),
        .DEAD_ZONE(1280),
        .STATE_WIDTH(3)
    ) inst_pwm_m2 (
        .clk(clk),
        .rst(reset),
        .speed_in(PWM_M2),
        .speed_oe(m2_oe),
        .pwm_out(pwm_2_out),
        .busy(m2_busy)
    );

    bb_pwm #(
        .MAX_SPEED(65536),
        .MIN_SPEED(256),
        .ACC(2560),
        .DEAD_ZONE(1280),
        .STATE_WIDTH(3)
    ) inst_pwm_m3 (
        .clk(clk),
        .rst(reset),
        .speed_in(PWM_M3),
        .speed_oe(m3_oe),
        .pwm_out(pwm_3_out),
        .busy(m3_busy)
    );

    bb_pwm #(
        .MAX_SPEED(65536),
        .MIN_SPEED(256),
        .ACC(2560),
        .DEAD_ZONE(1280),
        .STATE_WIDTH(3)
    ) inst_pwm_m4 (
        .clk(clk),
        .rst(reset),
        .speed_in(PWM_M4),
        .speed_oe(m4_oe),
        .pwm_out(pwm_4_out),
        .busy(m4_busy)
    );

    // 三段式状态机定义
    typedef enum reg [2:0] {IDLE, MPU_CAPTURE, CURRENT, TARGET, PID_CONTROL, PWM_OUT, ERROR} state_t;

// 三段式状态机  /////////////////////////////////////////////////////////////////

    typedef enum reg [2:0] {
        IDLE, MPU_CAPTURE, CONTROL, PWM_OUT, ERROR
    } state_t;
    state_t state, next_state;

    // 状态机时序逻辑
    always @(posedge clk or posedge reset) begin
        if (reset) state <= IDLE;
        else state <= next_state;
    end

    // 状态机组合逻辑
    always @(*) begin
        case (state)
            IDLE: next_state = mpu_init_done ? MPU_CAPTURE : IDLE;
            MPU_CAPTURE: next_state = capture_done ? CURRENT : MPU_CAPTURE;
            CURRENT: next_state = posture_is_confirmed ? TARGET : CURRENT;
            TARGET: next_state = PID_CONTROL;
            PID_CONTROL: next_state = PWM_OUT;
            PWM_OUT: next_state = MPU_CAPTURE;
            ERROR: next_state = IDLE;  // 错误状态返回IDLE
            IDLE: next_state = mpu_init_done ? MPU_CAPTURE : IDLE;//在这里初始化
            MPU_CAPTURE: next_state = capture_done ? PID_CONTROL : MPU_CAPTURE;//获取12byte的数据
            //使用加速度计算两个角度
            CURRENT: next_state = posture_is_confirmed ? TARGET : ACCEL_DONE;
            TARGET: next_state = PID_CONTROL;
            PID_CONTROL: next_state = PWM_OUT;
            PWM_OUT: next_state = MPU_CAPTURE;
            ERROR:;
            default: next_state = IDLE;
        endcase
    end

    // MPU数据采集和姿态计算
    reg [7:0] mpu_data_packed [0:11];  // MPU原始数据包
    reg [3:0] byte_counter;            // 数据字节计数器
    reg capture_done;                  // 数据采集完成标志
    reg posture_is_confirmed;          // 姿态计算完成标志
    reg [15:0] pitch_gyro, roll_gyro, yaw_gyro;  // 陀螺仪积分角度
    reg [15:0] current_pitch, current_roll;      // 当前姿态
    parameter signed [15:0] ALPHA = 98;          // 互补滤波系数 (0.98 * 100)
    parameter dt = 16'd1;                        // 假设时间步长（需根据实际采样率调整）

    // 加速度和陀螺仪数据转换
    assign ax = {mpu_data_packed[0], mpu_data_packed[1]} * 10000 / 16384;  // 加速度X轴
    assign ay = {mpu_data_packed[2], mpu_data_packed[3]} * 10000 / 16384;  // 加速度Y轴
    assign az = {mpu_data_packed[4], mpu_data_packed[5]} * 10000 / 16384;  // 加速度Z轴
    assign gyro_x = {mpu_data_packed[6], mpu_data_packed[7]} * 10000 / 16384;  // 陀螺仪X轴
    assign gyro_y = {mpu_data_packed[8], mpu_data_packed[9]} * 10000 / 16384;  // 陀螺仪Y轴
    assign gyro_z = {mpu_data_packed[10], mpu_data_packed[11]} * 10000 / 16384;// 陀螺仪Z轴

    // PID相关信号
    reg [15:0] PWM_base;              // 基础PWM值
    reg [15:0] error_height;          // 高度误差
    reg [15:0] error_pitch, error_roll, error_yaw;  // 姿态误差
    reg [15:0] Integral_Pitch_error, Integral_Roll_error, Integral_Yaw_error;  // 积分项
    reg [15:0] Derivative_Pitch_error, Derivative_Roll_error, Derivative_Yaw_error;  // 微分项
    reg [15:0] prev_error_pitch, prev_error_roll, prev_error_yaw;  // 上一次误差
    parameter Kp_pitch = 16'd100, Ki_pitch = 16'd10, Kd_pitch = 16'd50;  // PID参数（示例值）
    parameter Kp_roll = 16'd100, Ki_roll = 16'd10, Kd_roll = 16'd50;
    parameter Kp_yaw = 16'd80, Ki_yaw = 16'd5, Kd_yaw = 16'd30;

    // 主状态机输出逻辑
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mpu_init <= 1;
            mpu_transfer <= 0;
            byte_counter <= 0;
            capture_done <= 0;
            posture_is_confirmed <= 0;
            pitch_gyro <= 0;
            roll_gyro <= 0;
            yaw_gyro <= 0;
            prev_error_pitch <= 0;
            prev_error_roll <= 0;
            prev_error_yaw <= 0;
            Integral_Pitch_error <= 0;
            Integral_Roll_error <= 0;
            Integral_Yaw_error <= 0;
            Derivative_Pitch_error <= 0;
            Derivative_Roll_error <= 0;
            Derivative_Yaw_error <= 0;
            PWM_M1 <= 0;
            PWM_M2 <= 0;
            PWM_M3 <= 0;
            PWM_M4 <= 0;
            m1_oe <= 0;
            m2_oe <= 0;
            m3_oe <= 0;
            m4_oe <= 0;
        end else begin
            case (state)
                IDLE: begin
                    mpu_init <= 0;  // 初始化完成时关闭初始化信号
                end
                MPU_CAPTURE: begin
                    mpu_transfer <= 1;
                    if (mpu_data_oe) begin
                        if (byte_counter == 11) begin
                            byte_counter <= 0;
                            capture_done <= 1;
                            angle_start <= 1;
                            mpu_transfer <= 0;
                            // 陀螺仪角度积分
                            pitch_gyro <= pitch_gyro + gyro_x * dt;
                            roll_gyro <= roll_gyro + gyro_y * dt;
                            yaw_gyro <= yaw_gyro + gyro_z * dt;
                        end else begin
                            byte_counter <= byte_counter + 1;
                            mpu_data_packed[byte_counter] <= mpu_data;
                        end
                    end
                end
                CURRENT: begin
                    capture_done <= 0;
                    angle_start <= 0;
                    if (angle_done) begin
                        // 互补滤波计算当前姿态
                        current_pitch <= (ALPHA * pitch_gyro + (100 - ALPHA) * pitch_accel) / 100;
                        current_roll <= (ALPHA * roll_gyro + (100 - ALPHA) * roll_accel) / 100;
                        posture_is_confirmed <= 1;
                    end
                end
                TARGET: begin
                    posture_is_confirmed <= 0;
                    // 计算误差
                    error_height <= target_height - current_height;
                    error_pitch <= target_pitch - current_pitch;
                    error_roll <= target_roll - current_roll;
                    error_yaw <= target_yaw - yaw_gyro;
                    PWM_base <= is_move ? PWM_BASE + TILT_CMP : PWM_BASE;

                    // 更新PID积分项和微分项
                    Integral_Pitch_error <= Integral_Pitch_error + error_pitch;
                    Integral_Roll_error <= Integral_Roll_error + error_roll;
                    Integral_Yaw_error <= Integral_Yaw_error + error_yaw;
                    Derivative_Pitch_error <= error_pitch - prev_error_pitch;
                    Derivative_Roll_error <= error_roll - prev_error_roll;
                    Derivative_Yaw_error <= error_yaw - prev_error_yaw;

                    // 保存当前误差为下一次使用
                    prev_error_pitch <= error_pitch;
                    prev_error_roll <= error_roll;
                    prev_error_yaw <= error_yaw;
                end
                PID_CONTROL: begin
                    m1_oe <= 1;
                    m2_oe <= 1;
                    m3_oe <= 1;
                    m4_oe <= 1;
                    // 计算四个电机的PWM值
                    PWM_M1 <= PWM_base - (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                                       - (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
                                       - (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
                    PWM_M2 <= PWM_base - (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                                       + (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
                                       + (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
                    PWM_M3 <= PWM_base + (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                                       - (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
                                       + (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
                    PWM_M4 <= PWM_base + (Kp_pitch * error_pitch + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                                       + (Kp_roll * error_roll + Ki_roll * Integral_Roll_error + Kd_roll * Derivative_Roll_error)
                                       - (Kp_yaw * error_yaw + Ki_yaw * Integral_Yaw_error + Kd_yaw * Derivative_Yaw_error);
                end
                PWM_OUT: begin
                    m1_oe <= 0;
                    m2_oe <= 0;
                    m3_oe <= 0;
                    m4_oe <= 0;
                end
            endcase
        end
    end


// 算法处理 与 信号输出
    
    // mpu采集3轴(byte_counter == 12)
    reg [7:0] mpu_data_packed [0:7];    // 完整数据（示例用Z轴加速度）
    reg [3:0] byte_counter;
    always @(posedge clk) begin
        if (state == MPU_CAPTURE && mpu_data_oe) begin
            if (byte_counter == 12) begin
                byte_counter <= 0; // Reset byte_counter
            end
            else begin
                byte_counter <= byte_counter + 1; // Increment byte_counter
                mpu_data_packed[byte_counter] <= mpu_data; // Store data
            end
        end
    end

    // PID_CONTROL------------------   
    // 角度计算
    // ax, ay, az 是物理加速度值（单位 g 或 m/s²）
    wire signed [31:0] ax = {mpu_data_packed[0:1]} * 10000 / 16384;  // 1g = 10000
    wire signed [31:0] ay = {mpu_data_packed[2:3]} * 10000 / 16384;
    wire signed [31:0] az = {mpu_data_packed[4:5]} * 10000 / 16384;
    pitch_accel = cordic_roll(ax, ay, az);
    roll_accel =  cordic_roll(ax, ay, az);

    wire signed [31:0] gyro_x = {mpu_data_packed[6:7]} * 10000 / 16384;  // 1弧度 = 10000
    wire signed [31:0] gyro_y = {mpu_data_packed[8:9]} * 10000 / 16384;
    wire signed [31:0] gyro_z = {mpu_data_packed[10:11]} * 10000 / 16384;
    pitch_gyro <= pitch_gyro + gyro_x * dt;
    roll_gyro  <= roll_gyro + gyro_y * dt;
    yaw_gyro   <= yaw_gyro + gyro_z * dt;

    // 五个输出：pitch_accel、roll_accel、
    //     pitch_gyro、roll_gyro、yaw_gyro

    // 互补滤波
    parameter signed [15:0] ALPHA = 98; // 0.98 (放大100倍避免小数计算)

    pitch <= (ALPHA * pitch_gyro + (100 - ALPHA) * pitch_accel) / 100;
    roll  <= (ALPHA * roll_gyro + (100 - ALPHA) * roll_accel) / 100;
    // 计算出error
    get_target();//串口发过来的数据会存在寄存器中，异步读取寄存器作为目标

    Pitch_error = pitch_target - pitch;
    roll_error = roll_target - roll;
    Yaw_error = yaw_target - yaw_gyro;
    Height_error = Height_target - Height_azdt;

    cal_pwm_悬停(); // pwm_悬停 的值通过abs(Height_target - target_height) < 10 时候的值确定
    PWM_base = cal_pwm_悬停 + compensate(is_move);//这里的补偿是前后左右移动时候Z轴的重力分力会变，补偿保证不会掉落

    // pid
    PWM_M1 = PWM_base - (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error) 
                    - (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error) 
                    - (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error)

    PWM_M2 = PWM_base - (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                      + (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error)
                      + (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error)

    PWM_M3 = PWM_base + (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                      - (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error)
                      + (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error)

    PWM_M4 = PWM_base + (Kp_pitch * Pitch_error + Ki_pitch * Integral_Pitch_error + Kd_pitch * Derivative_Pitch_error)
                      + (Kp_roll * Roll_error   + Ki_roll * Integral_Roll_error   + Kd_roll * Derivative_Roll_error)
                      - (Kp_yaw * Yaw_error     + Ki_yaw * Integral_Yaw_error     + Kd_yaw * Derivative_Yaw_error)


endmodule

姿态计算:begin

    // set PWM_base = 待机、悬停、起飞
    // 修改Pitch_error的值 = 前后
    // 修改Roll_error的值 = 左右
    // 修改Yaw_error的值 = 旋转
    // pwm_悬停 的值通过abs(height - target_height) < 10 时候的值确定

    PWM_base = pwm_悬停 + （上下 or 前后左右补偿）：在移动时，倾斜会损失部分垂直升力，可能导致高度下降。
    // 上下 = 高度pid：(Kp_height * Height_error + Ki_height * Integral_Height_error + Kd_height * Derivative_Height_error)


end
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
    output pwm_out,    // PWM输出（单电机示例）

    input RxD
);

// rs232模块信号 /////////////////////////////////////////////////////////////////
    wire RxD_idle;
    wire RxD_endofpacket;
    wire RxD_data_ready;
    wire [7:0] RxD_data;

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

// cordic module //////////////////////////////////////////////////////////////
    cordic_angle #(
        .ITERATIONS(12)
    ) inst_angle (
        .clk(clk),
        .rst_n(reset),
        .x_in(mpu_data_packed[0:1]),
        .y_in(mpu_data_packed[2:3]),
        .z_in(mpu_data_packed[4:5]),
        .start(angle_start),
        .done(angle_done),
        .pitch(pitch),
        .roll(roll)
    )

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


// 三段式状态机  /////////////////////////////////////////////////////////////////

    typedef enum reg [2:0] {
        IDLE, MPU_CAPTURE, CONTROL, PWM_OUT, ERROR
    } state_t;
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



    // 算法处理 与 信号输出
    // mpu采集3轴(byte_counter == 12)
    reg [7:0] mpu_data_packed [0:11];    // 完整数据（示例用Z轴加速度）
    reg [3:0] byte_counter;
    always @(posedge clk) begin
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
                        yaw_gyro   <= yaw_gyro + gyro_z * dt;
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
                    pitch <= (ALPHA * pitch_gyro + (100 - ALPHA) * pitch_accel) / 100;//互补滤波
                    roll  <= (ALPHA * roll_gyro + (100 - ALPHA) * roll_accel) / 100;
                    posture_is_confirmed <= 1;
                end
            end
        endcase 
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
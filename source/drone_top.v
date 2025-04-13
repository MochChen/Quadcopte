// `include "mpu_top.v"
// `include "mpu_mid.v"
// `include "std_iic_master.v"
// `include "cordic.v"
// `include "cordic_angle.v"
// `include "pwm.v"
// `include "target_RS232.v"
// `include "async_receiver.v"

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
    input wire RxD               // 串口接收数据
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
                CMP_FILTER   = 3'h4,
                CAL_ERROR    = 3'h5,
                CAL_PID      = 3'h6,
                NEXT_CAPTURE = 3'h7;
    reg [2:0] state = IDLE;

    reg mpu_init_done_reg;
    reg [7:0] mpu_data_packed [13:0]; //一共14字节
    reg [3:0] byte_cnt = 0;
    reg [3:0] ALPHA = 99;
    reg dt = 1; //mpu采集的间隔时间,单位秒
    reg [23:0] cur_pitch_gyro;
    reg [23:0] cur_roll_gyro;
    reg [23:0] cur_yaw_gyro;
    always @(posedge clk) begin
        if (mpu_read_done) begin
            cur_pitch_gyro <= cur_pitch_gyro + dt * $signed({mpu_data_packed[8], mpu_data_packed[9]});
            cur_roll_gyro <= cur_roll_gyro + dt * $signed({mpu_data_packed[10], mpu_data_packed[11]});
            cur_yaw_gyro <= cur_yaw_gyro + dt * $signed({mpu_data_packed[12], mpu_data_packed[13]});
        end
    end
    reg [23:0] cur_pitch; // 积分满了会不会溢出?
    reg [23:0] cur_roll;
    reg [23:0] cur_yaw;
    reg [23:0] tgt_height;//由外部串口信号输入进行更新
    reg [23:0] tgt_pitch; //由外部串口信号输入进行更新
    reg [23:0] tgt_roll;  //由外部串口信号输入进行更新
    reg [23:0] tgt_yaw;   //由外部串口信号输入进行更新
    reg [23:0] pitch_error;
    reg [23:0] roll_error;
    reg [23:0] yaw_error;
    reg [23:0] pre_pitch_error;
    reg [23:0] pre_roll_error;
    reg [23:0] pre_yaw_error;
    reg [23:0] i_pitch_error;
    reg [23:0] i_roll_error;
    reg [23:0] i_yaw_error;
    reg [23:0] d_pitch_error;
    reg [23:0] d_roll_error;
    reg [23:0] d_yaw_error;
    reg [23:0] PWM_base; //和高度控制有关,起飞时候逐渐增大,到达指定高度并且稳定决定,逻辑不知道怎么实现
    reg [7:0] Kp = 100;
    reg [7:0] Ki = 1;
    reg [7:0] Kd = 1;


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
                        cur_pitch <= (ALPHA * cur_pitch_gyro + (100 - ALPHA) * cur_pitch_acc) / 100;
                        cur_roll  <= (ALPHA * cur_roll_gyro  + (100 - ALPHA) * cur_roll_acc) / 100;
                        cur_yaw   <= cur_yaw_gyro;
                        state <= CAL_ERROR;
                    end
                CAL_ERROR:
                    begin
                        // 误差
                        pitch_error <= tgt_pitch - cur_pitch;
                        roll_error <= tgt_roll - cur_roll;
                        yaw_error <= tgt_yaw - cur_yaw;

                        // 更新积分
                        i_pitch_error <= i_pitch_error + pitch_error;   //可能会溢出
                        i_roll_error <= i_roll_error + roll_error;
                        i_yaw_error <= i_yaw_error + yaw_error;
                        // 更新微分
                        d_pitch_error <= (pitch_error - pre_pitch_error) * 90 / 100; // 这里应该是除以dt,暂时用0.9代替
                        d_roll_error <= (roll_error - pre_roll_error) * 90 / 100;
                        d_yaw_error <= (yaw_error - pre_yaw_error) * 90 / 100;

                        // 保存当前误差为下一次使用
                        pre_pitch_error <= pitch_error;
                        pre_roll_error <= roll_error;
                        pre_yaw_error <= yaw_error;

                        state <= CAL_PID;
                    end
                CAL_PID:
                    begin
                        state <= NEXT_CAPTURE;
                        duty_1_oe <= 1;
                        duty_2_oe <= 1;
                        duty_3_oe <= 1;
                        duty_3_oe <= 1;
                        pwm_duty_1 <= PWM_base  - (Kp * pitch_error + Ki * i_pitch_error + Kd * d_pitch_error)
                                                - (Kp * roll_error  + Ki * i_roll_error  + Kd * d_roll_error)
                                                - (Kp * yaw_error   + Ki * i_yaw_error   + Kd * d_yaw_error);

                        pwm_duty_2 <= PWM_base  - (Kp * pitch_error + Ki * i_pitch_error + Kd * d_pitch_error)
                                                + (Kp * roll_error  + Ki * i_roll_error  + Kd * d_roll_error)
                                                + (Kp * yaw_error   + Ki * i_yaw_error   + Kd * d_yaw_error);

                        pwm_duty_3 <= PWM_base  + (Kp * pitch_error + Ki * i_pitch_error + Kd * d_pitch_error)
                                                - (Kp * roll_error  + Ki * i_roll_error  + Kd * d_roll_error)
                                                + (Kp * yaw_error   + Ki * i_yaw_error   + Kd * d_yaw_error);
                                            
                        pwm_duty_4 <= PWM_base  + (Kp * pitch_error + Ki * i_pitch_error + Kd * d_pitch_error)
                                                + (Kp * roll_error  + Ki * i_roll_error  + Kd * d_roll_error)
                                                - (Kp * yaw_error   + Ki * i_yaw_error   + Kd * d_yaw_error);
                    end
                NEXT_CAPTURE:
                    begin
                        duty_1_oe <= 0;
                        duty_2_oe <= 0;
                        duty_3_oe <= 0;
                        duty_4_oe <= 0;
                        if (signal_INT) state <= MPU_CAPTURE;
                    end
                default: state <= IDLE;
            endcase
        end
    end
    
    reg [15:0] pwm_duty_1;
    reg duty_1_oe;
    wire pwm_1;
    wire busy_1;

    pwm PWM_1 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_1),
        .speed_oe(duty_1_oe),
        .pwm(pwm_1),
        .busy(busy_1)
    );

    reg [15:0] pwm_duty_2;
    reg duty_2_oe;
    wire pwm_2;
    wire busy_2;

    pwm PWM_2 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_2),
        .speed_oe(duty_2_oe),
        .pwm(pwm_2),
        .busy(busy_2)
    );

    reg [15:0] pwm_duty_3;
    reg duty_3_oe;
    wire pwm_3;
    wire busy_3;

    pwm PWM_3 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_3),
        .speed_oe(duty_3_oe),
        .pwm(pwm_3),
        .busy(busy_3)
    );

    reg [15:0] pwm_duty_4;
    reg duty_4_oe;
    wire pwm_4;
    wire busy_4;

    pwm PWM_4 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_4),
        .speed_oe(duty_4_oe),
        .pwm(pwm_4),
        .busy(busy_4)
    );

    // 目标指令
    wire target_renew;
    wire [15:0] target_height;
    wire [15:0] target_pitch;
    wire [15:0] target_roll;
    wire [15:0] target_yaw;

    target_RS232 target_ins(
        .clk(clk),
        .rst_n(rst_n),
        .RxD(RxD),
        .target_renew(target_renew),
        .target_height(target_height),
        .target_pitch(target_pitch),
        .target_roll(target_roll),
        .target_yaw(target_yaw)
    );
    always @(posedge clk) 
        if (target_renew && (state != CAL_ERROR)) begin
            tgt_height <= target_height;
            tgt_pitch <= target_pitch;
            tgt_roll <= target_roll;
            tgt_yaw <= target_yaw;
        end

    
endmodule



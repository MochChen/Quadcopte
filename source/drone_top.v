// `include "mpu_top.v"
// `include "mpu_mid.v"
// `include "std_iic_master.v"
// `include "cordic.v"
// `include "cordic_angle.v"
// `include "pwm.v"
// `include "target_RS232.v"
// `include "async_receiver.v"
// `include "cal_gyro.v"
// `include "cmp_filter.v"
// `include "cal_error.v"
// `include "cal_pid.v"

module drone_top #(
    parameter PWM_BASE = 16'd30000,  // 悬停时的基础PWM值
    parameter TILT_CMP = 16'd5000    // 倾斜时的升力补偿
)(
    // input wire clk,              // 50MHz主时钟
    input wire FCLK_CLK0_0,         // 50MHz主时钟
    input wire FCLK_RESET0_N_0,     // 低有效复位
    output wire scl,             // I2C时钟
    inout wire sda,              // I2C数据
    input wire signal_INT,
    output wire pwm_1,       // PWM输出至电机1
    output wire pwm_2,       // PWM输出至电机2
    output wire pwm_3,       // PWM输出至电机3
    output wire pwm_4,       // PWM输出至电机4
    input wire RxD           // 串口接收数据
);
    wire clk = FCLK_CLK0_0;
    wire rst_n = FCLK_RESET0_N_0;


    parameter   IDLE         = 4'h0,
                INIT_MPU     = 4'h1,
                MPU_CAPTURE  = 4'h2,
                CRD_PITCH    = 4'h3,
                CRD_ROLL     = 4'h4,
                CMP_FILTER   = 4'h5,
                CAL_ERROR    = 4'h6,
                CAL_PID      = 4'h7,
                PWM_OUT      = 4'h8,
                NEXT_CAPTURE = 4'h9,
                SYS_ERROR    = 4'ha;
    reg [3:0] state = IDLE;

    reg mpu_init_done_reg;
    reg init_error;
    reg [7:0] mpu_data_packed [13:0]; //一共14字节
    reg [13:0] init_error_cnt;
    reg [3:0] byte_cnt;

    reg [15:0] delay_cnt;//1ms计时，50MHz

    reg [23:0] PWM_base = 16'sd1000; //和高度控制有关,起飞时候逐渐增大,到达指定高度并且稳定决定,逻辑不知道怎么实现
    reg [7:0] Kp = 100;
    reg [7:0] Ki = 1;
    reg [7:0] Kd = 1;
    
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            mpu_init_done_reg <= 0;
            delay_cnt <= 0;
            init_error_cnt <= 0;
            byte_cnt <= 0;
        end else begin
            case (state)
                IDLE: //0
                    begin
                        if (!mpu_init_done_reg) begin
                            state <= INIT_MPU;
                            mpu_init_start <= 1;
                        end else begin
                            if (delay_cnt == 49999) begin
                                state <= MPU_CAPTURE;
                                mpu_read_start <= 1;
                                delay_cnt <= 0;
                            end else begin
                                state <= IDLE;
                                delay_cnt <= delay_cnt + 1;
                            end
                        end
                        init_error_cnt <= 0;
                    end
                INIT_MPU: //1
                    begin
                        mpu_init_start <= 0;
                        if (mpu_init_done) begin
                            mpu_init_done_reg <= 1;
                            state <= IDLE;
                        end else begin
                            init_error_cnt <= init_error_cnt + 1;
                            if (&init_error_cnt) begin
                                init_error <= 1;
                                init_error_cnt <= 0;
                                state <= SYS_ERROR;
                            end
                        end
                    end
                MPU_CAPTURE://2
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
                CRD_PITCH://3
                    begin
                        pitch_start <= 0;
                        if (pitch_done) begin
                            cur_pitch_acc <= pitch_acc;
                            state <= CRD_ROLL;
                            roll_start <= 1;
                        end
                    end
                CRD_ROLL://4
                    begin
                        roll_start <= 0;
                        if (roll_done) begin
                            cur_roll_acc <= roll_acc;
                            state <= CMP_FILTER;
                        end
                    end
                CMP_FILTER://5
                    begin
                        cmp_filter_en <= 1;
                        state <= CAL_ERROR;
                    end
                CAL_ERROR://6
                    begin
                        cal_error_en <= 1;
                        state <= CAL_PID;
                    end
                CAL_PID://7
                    begin
                        cal_error_en <= 0;
                        cal_pid_en <= 1;
                        state <= PWM_OUT;
                    end
                PWM_OUT://8
                    begin
                        cal_pid_en <= 0;
                        state <= NEXT_CAPTURE;
                        duty_1_oe <= 1;
                        duty_2_oe <= 1;
                        duty_3_oe <= 1;
                        duty_4_oe <= 1;
                    end
                NEXT_CAPTURE://9
                    begin
                        duty_1_oe <= 0;
                        duty_2_oe <= 0;
                        duty_3_oe <= 0;
                        duty_4_oe <= 0;
                        if (delay_cnt == 49999) begin //决定了控制频率
                            mpu_read_start <= 1;
                            delay_cnt <= 0;
                            state <= MPU_CAPTURE;
                        end else begin
                            state <= NEXT_CAPTURE;
                            delay_cnt <= delay_cnt + 1;
                        end
                    end
                SYS_ERROR:
                    begin
                        if (!rst_n) state <= IDLE;
                    end
                default: state <= IDLE;
            endcase
        end
    end

    // MPU模块实例化
    reg mpu_init_start;
    reg mpu_read_start;
    wire mpu_init_done;
    wire mpu_read_done;
    wire mpu_data_avalid;
    wire [7:0] mpu_data;
    wire [2:0] iic_state;
    wire sda_en;
    wire sda_out;
    wire sda_in;

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
        .sda_en(sda_en),
        .sda_out(sda_out),
        .sda_in(sda_in),
        .iic_state(iic_state)
    );

    assign sda = sda_en ? sda_out : 1'bz; //顶层使用三态门inout
    assign sda_in = sda;


    // 用加速度计算角度模块实例化
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
    
    // 目标指令
    wire target_renew;
    wire signed [23:0] target_height;
    wire signed [23:0] target_pitch;
    wire signed [23:0] target_roll;
    wire signed [23:0] target_yaw;

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
    reg signed [23:0] tgt_height;//由外部串口信号输入进行更新
    reg signed [23:0] tgt_pitch; //由外部串口信号输入进行更新
    reg signed [23:0] tgt_roll;  //由外部串口信号输入进行更新
    reg signed [23:0] tgt_yaw;   //由外部串口信号输入进行更新
    always @(posedge clk) 
        if (target_renew && (state != CAL_ERROR)) begin
            tgt_height <= target_height;
            tgt_pitch <= target_pitch;
            tgt_roll <= target_roll;
            tgt_yaw <= target_yaw;
        end

    // 角速度积分
    wire [23:0] cur_pitch_gyro;
    wire [23:0] cur_roll_gyro;
    wire [23:0] cur_yaw_gyro;
    
    cal_gyro cal_gyro_ins (
        .clk(clk),
        .rst_n(rst_n),
        .cal_gyro_en(mpu_read_done),
        .mpu_data_packed_8(mpu_data_packed[8]),
        .mpu_data_packed_9(mpu_data_packed[9]),
        .mpu_data_packed_10(mpu_data_packed[10]),
        .mpu_data_packed_11(mpu_data_packed[11]),
        .mpu_data_packed_12(mpu_data_packed[12]),
        .mpu_data_packed_13(mpu_data_packed[13]),
        .cur_pitch_gyro(cur_pitch_gyro),
        .cur_roll_gyro(cur_roll_gyro),
        .cur_yaw_gyro(cur_yaw_gyro)
    );
    
    // 互补滤波
    reg cmp_filter_en;

    wire signed [23:0] cur_pitch;
    wire signed [23:0] cur_roll;
    wire signed [23:0] cur_yaw;
    
    cmp_filter cmp_filter_ins (
        .clk(clk),
        .rst_n(rst_n),
        .cmp_filter_en(cmp_filter_en),
        .cur_pitch_gyro(cur_pitch_gyro),
        .cur_roll_gyro(cur_roll_gyro),
        .cur_yaw_gyro(cur_yaw_gyro),
        .cur_pitch_acc({{8{cur_pitch_acc[15]}}, cur_pitch_acc}),
        .cur_roll_acc({{8{cur_roll_acc[15]}}, cur_roll_acc}),
        .cur_pitch(cur_pitch),
        .cur_roll(cur_roll),
        .cur_yaw(cur_yaw)
    );
    
    // cal error
    reg cal_error_en;

    wire [23:0] pitch_error;
    wire [23:0] roll_error;
    wire [23:0] yaw_error;
    wire [23:0] i_pitch_error;
    wire [23:0] i_roll_error;
    wire [23:0] i_yaw_error;
    wire [23:0] d_pitch_error;
    wire [23:0] d_roll_error;
    wire [23:0] d_yaw_error;
    
    cal_error cal_error_ins (
        .clk(clk),
        .rst_n(rst_n),
        .cal_error_en(cal_error_en),
        .tgt_pitch(tgt_pitch),
        .tgt_roll(tgt_roll),
        .tgt_yaw(tgt_yaw),
        .cur_pitch(cur_pitch),
        .cur_roll(cur_roll),
        .cur_yaw(cur_yaw),
        .pitch_error(pitch_error),
        .roll_error(roll_error),
        .yaw_error(yaw_error),
        .i_pitch_error(i_pitch_error),
        .i_roll_error(i_roll_error),
        .i_yaw_error(i_yaw_error),
        .d_pitch_error(d_pitch_error),
        .d_roll_error(d_roll_error),
        .d_yaw_error(d_yaw_error)
    );

    // cal pid
    reg cal_pid_en;

    wire [15:0] pwm_duty_1;
    wire [15:0] pwm_duty_2;
    wire [15:0] pwm_duty_3;
    wire [15:0] pwm_duty_4;

    cal_pid cal_pid_ins (
        .clk(clk),
        .rst_n(rst_n),
        .cal_pid_en(cal_pid_en),
        .PWM_base(PWM_base),
        .pitch_error(pitch_error),
        .roll_error(roll_error),
        .yaw_error(yaw_error),
        .i_pitch_error(i_pitch_error),
        .i_roll_error(i_roll_error),
        .i_yaw_error(i_yaw_error),
        .d_pitch_error(d_pitch_error),
        .d_roll_error(d_roll_error),
        .d_yaw_error(d_yaw_error),
        .pwm_duty_1(pwm_duty_1),
        .pwm_duty_2(pwm_duty_2),
        .pwm_duty_3(pwm_duty_3),
        .pwm_duty_4(pwm_duty_4)
    );

    // pwm输出信号
    reg duty_1_oe;
    reg duty_2_oe;
    reg duty_3_oe;
    reg duty_4_oe;
    wire busy_1;
    wire busy_2;
    wire busy_3;
    wire busy_4;

    pwm PWM_1 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_1),
        .speed_oe(duty_1_oe),
        .pwm(pwm_1),
        .busy(busy_1)
    );

    pwm PWM_2 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_2),
        .speed_oe(duty_2_oe),
        .pwm(pwm_2),
        .busy(busy_2)
    );

    pwm PWM_3 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_3),
        .speed_oe(duty_3_oe),
        .pwm(pwm_3),
        .busy(busy_3)
    );

    pwm PWM_4 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_4),
        .speed_oe(duty_4_oe),
        .pwm(pwm_4),
        .busy(busy_4)
    );


endmodule



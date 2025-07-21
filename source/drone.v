// `include "register_map.v"
// `include "mpu.v"
// `include "iic.v"
// `include "cordic.v"
// `include "cordic_angle.v"
// `include "pwm.v"
// `include "cal_gyro.v"
// `include "filter.v"
// `include "cal_error.v"
// `include "pid.v"

module drone (
    input  wire clk,                // 50MHz主时�?
    input  wire rst_n,              // 低有效复�?
    // mpu接口
    output wire scl,                // I2C时钟
    inout  wire sda,                // I2C数据

    // axi接口
    input  wire [31:0]          axi_din_32,             // 接收32bit
    input  wire                 axi_din_valid,          // 数据有效信号
    output wire [63:0]          axi_out_64H,            // 发�?64bit_H
    output wire [63:0]          axi_out_64L,

    output wire pwm_1,              // PWM输出至电�?1
    output wire pwm_2,              // PWM输出至电�?2
    output wire pwm_3,              // PWM输出至电�?3
    output wire pwm_4               // PWM输出至电�?4

);

/**************************************************************************************\
                                    MPU模块
\**************************************************************************************/
    reg mpu_init_start;

    reg mpu_read_start;
    wire mpu_read_done;
    wire [111:0] mpu_data_pack;             // 14字节

    wire sda_oe;
    wire sda_out;
    wire sda_in;
    assign sda = sda_oe ? sda_out : 1'bz;   //顶层使用三态门inout
    assign sda_in = sda;

    wire [7:0]  mpu_device_addr;
    wire [7:0]  mpu_register_addr;
    wire [7:0]  mpu_register_data;
    wire [7:0]  mpu_read_start_addr;

    reg mpu_error_reset;
    wire mpu_error;

    mpu ins_mpu (
        .clk(clk),
        .rst_n(rst_n),

        .mpu_init_start(mpu_init_start),

        .mpu_read_start(mpu_read_start),
        .mpu_read_done(mpu_read_done),
        .mpu_data_pack(mpu_data_pack),

        .scl(scl),
        .sda_oe(sda_oe),
        .sda_out(sda_out),
        .sda_in(sda_in),

        .mpu_device_addr(mpu_device_addr),
        .mpu_register_addr(mpu_register_addr),
        .mpu_register_data(mpu_register_data),
        .mpu_read_start_addr(mpu_read_start_addr),

        .mpu_error_reset(mpu_error_reset),
        .mpu_error(mpu_error)
    );




/**************************************************************************************\
                                用加速度计算角度
\**************************************************************************************/
    wire signed [15:0] ax = $signed({mpu_data_pack[0*8 +: 8], mpu_data_pack[1*8 +: 8]}) - 1200;
    wire signed [15:0] ay = $signed({mpu_data_pack[2*8 +: 8], mpu_data_pack[3*8 +: 8]}) + 600;
    wire signed [15:0] az = $signed({mpu_data_pack[4*8 +: 8], mpu_data_pack[5*8 +: 8]}) + 150;

    reg pitch_start;
    wire pitch_done;
    wire [15:0] crda_angle_pitch;
    (* keep *) reg [15:0] cur_pitch_acc;

    cordic_angle ins_pitch(
        .clk(clk),
        .rst_n(rst_n),
        .x(ax),
        .y(ay),
        .z(az),
        .cdra_start(pitch_start),
        .cdra_done(pitch_done),
        .crda_angle(crda_angle_pitch)
    );

    reg roll_start;
    wire roll_done;
    wire [15:0] crda_angle_roll;
    (* keep *) reg [15:0] cur_roll_acc;

    cordic_angle ins_roll (
        .clk(clk),
        .rst_n(rst_n),
        .x(ay), //更换位置就是roll
        .y(ax),
        .z(az),
        .cdra_start(roll_start),
        .cdra_done(roll_done),
        .crda_angle(crda_angle_roll)
    );
    
/**************************************************************************************\
                                    寄存器刷�?
\**************************************************************************************/

    wire signed [15:0] pwm_base;
    wire signed [15:0] tilt_cmp;
    wire signed [7:0]  use_INT;
    wire [7:0] error_reset;

    wire [15:0] tgt_height;
    wire [15:0] tgt_pitch;
    wire [15:0] tgt_roll;
    wire [15:0] tgt_yaw;

    wire [23:0] action_reg;
    wire [7:0]  alpha;
    wire [23:0] max_speed;
    wire [23:0] min_speed;
    wire [7:0] KP;
    wire [7:0] KI; 
    wire [7:0] KD;

    reg [7:0]  error_code;

    register_map ins_map (
        .clk                (clk),
        .rst_n              (rst_n),

        .axi_din_32         (axi_din_32),
        .axi_din_valid      (axi_din_valid),
        .axi_out_64H        (axi_out_64H),
        .axi_out_64L        (axi_out_64L),

        .pwm_base           (pwm_base),
        .tilt_cmp           (tilt_cmp),
        .use_INT            (use_INT),
        .error_reset        (error_reset),

        .mpu_device_addr    (mpu_device_addr),
        .mpu_register_addr  (mpu_register_addr),
        .mpu_register_data  (mpu_register_data),
        .mpu_read_start_addr(mpu_read_start_addr),
        
        .tgt_height         (tgt_height),
        .tgt_pitch          (tgt_pitch),
        .tgt_roll           (tgt_roll),
        .tgt_yaw            (tgt_yaw),

        .action_reg         (action_reg),
        .alpha              (alpha),
        .max_speed          (max_speed),
        .min_speed          (min_speed),
        .KP                 (KP),
        .KI                 (KI),
        .KD                 (KD),

        .error_code         (error_code),
        .mpu_read_done      (mpu_read_done),
        .mpu_data_pack      (mpu_data_pack)
    );

/**************************************************************************************\
                                    角速度计算
\**************************************************************************************/
    wire signed [15:0] pitch_gyro = $signed({mpu_data_pack[8*8  +: 8], mpu_data_pack[9*8  +: 8]}) + 500;
    wire signed [15:0] roll_gyro  = $signed({mpu_data_pack[10*8 +: 8], mpu_data_pack[11*8 +: 8]}) - 107;
    wire signed [15:0] yaw_gyro   = $signed({mpu_data_pack[12*8 +: 8], mpu_data_pack[13*8 +: 8]}) - 95;
    
    wire signed [15:0] cur_pitch_gyro;
    wire signed [15:0] cur_roll_gyro;
    wire signed [15:0] cur_yaw_gyro;
    
    cal_gyro ins_gyro (
        .clk(clk),
        .rst_n(rst_n),

        .cal_gyro_oe(mpu_read_done),

        .pitch_gyro(pitch_gyro),
        .roll_gyro(roll_gyro),
        .yaw_gyro(yaw_gyro),

        .cur_pitch_gyro (cur_pitch_gyro),
        .cur_roll_gyro  (cur_roll_gyro),
        .cur_yaw_gyro   (cur_yaw_gyro)
    );
    
/**************************************************************************************\
                                    互补滤波
\**************************************************************************************/

    reg filter_en;

    wire signed [15:0] cur_pitch;
    wire signed [15:0] cur_roll;
    wire signed [15:0] cur_yaw;

    // wire signed [15:0] pitch_feedback;
    // wire signed [15:0] roll_feedback;
    // wire signed [15:0] yaw_feedback;
    
    filter ins_filter (
        .clk(clk),
        .rst_n(rst_n),
        .filter_en  (filter_en),

        .cur_pitch_gyro (cur_pitch_gyro),
        .cur_roll_gyro  (cur_roll_gyro),
        .cur_yaw_gyro   (cur_yaw_gyro),

        .cur_pitch_acc  (cur_pitch_acc),
        .cur_roll_acc   (cur_roll_acc),

        .cur_pitch  (cur_pitch),
        .cur_roll   (cur_roll),
        .cur_yaw    (cur_yaw),

        // .pitch_feedback(pitch_feedback),
        // .roll_feedback(roll_feedback),
        // .yaw_feedback(yaw_feedback),

        .alpha(alpha)
    );
    
/**************************************************************************************\
                                    误差
\**************************************************************************************/
    reg cal_error_en;

    wire signed [15:0] pitch_error;
    wire signed [15:0] roll_error;
    wire signed [15:0] yaw_error;

    wire signed [15:0] i_pitch_error;
    wire signed [15:0] i_roll_error;
    wire signed [15:0] i_yaw_error;

    wire signed [15:0] d_pitch_error;
    wire signed [15:0] d_roll_error;
    wire signed [15:0] d_yaw_error;
    
    cal_error ins_cal_error (
        .clk(clk),
        .rst_n(rst_n),

        .cal_error_en(cal_error_en),

        .tgt_pitch(tgt_pitch),
        .tgt_roll(tgt_roll),
        .tgt_yaw(tgt_yaw),

        .tgt_height(tgt_height),
        .cur_pitch(cur_pitch),
        .cur_roll(cur_roll),
        .cur_yaw(cur_yaw),

        .pitch_gyro(pitch_gyro),
        .roll_gyro(roll_gyro),
        .yaw_gyro(yaw_gyro),

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

/**************************************************************************************\
                                    PID
\**************************************************************************************/
    reg cal_pid_en;

    wire [15:0] pwm_duty_1;
    wire [15:0] pwm_duty_2;
    wire [15:0] pwm_duty_3;
    wire [15:0] pwm_duty_4;

    pid ins_pid (
        .clk(clk),
        .rst_n(rst_n),

        .cal_pid_en(cal_pid_en),
        .pwm_base(pwm_base),

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
        .pwm_duty_4(pwm_duty_4),

        .KP(KP),
        .KI(KI),
        .KD(KD)
    );


/**************************************************************************************\
                                    pwm输出
\**************************************************************************************/
    reg duty_1_oe;
    reg duty_2_oe;
    reg duty_3_oe;
    reg duty_4_oe;

    pwm ins_pwm_1 (
        .clk(clk),
        .rst_n(rst_n),

        .speed_in(pwm_duty_1),
        .speed_oe(duty_1_oe),
        .pwm(pwm_1),
        .max_speed(max_speed),
        .min_speed(min_speed)
    );

    pwm ins_pwm_2 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_2),
        .speed_oe(duty_2_oe),
        .pwm(pwm_2),
        .max_speed(max_speed),
        .min_speed(min_speed)
    );

    pwm ins_pwm_3 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_3),
        .speed_oe(duty_3_oe),
        .pwm(pwm_3),
        .max_speed(max_speed),
        .min_speed(min_speed)
    );

    pwm ins_pwm_4 (
        .clk(clk),
        .rst_n(rst_n),
        .speed_in(pwm_duty_4),
        .speed_oe(duty_4_oe),
        .pwm(pwm_4),
        .max_speed(max_speed),
        .min_speed(min_speed)
    );

/**************************************************************************************\
                                    主程�?
\**************************************************************************************/

    parameter   IDLE         = 4'h0,
                INIT_MPU     = 4'h1,
                MPU_CAPTURE  = 4'h2,
                PITCH_CAL    = 4'h3,
                ROLL_CAL     = 4'h4,
                CMP_FILTER   = 4'h5,
                CAL_ERROR    = 4'h6,
                CAL_PID      = 4'h7,
                PWM_OUT      = 4'h8,
                NEXT_CAPTURE = 4'h9,
                SYS_ERROR    = 4'ha;
    reg [3:0] state = IDLE;

    reg init_mpu;               // 如果mpu初始化成功就拉高
    reg [19:0] delay_cnt;
    localparam integer CONTROL_CYCLE = 150000 - 1;// 50000/1ms，当时钟50MHz

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_mpu <= 0;
            delay_cnt <= 0;
            mpu_error_reset <= 0;
            error_code <= 0;
            cur_pitch_acc <= 0;
            cur_roll_acc <= 0;
            mpu_init_start <= 0;
            pitch_start <= 0;
            mpu_read_start <= 0;
            roll_start <= 0;
            filter_en <= 0;
            cal_error_en <= 0;
            cal_pid_en <= 0;
            duty_1_oe <= 0;
            duty_2_oe <= 0;
            duty_3_oe <= 0;
            duty_4_oe <= 0;

            state <= IDLE;
        end else begin
            case (state)
                IDLE:           // state = 0
                    begin
                        mpu_error_reset <= 0;           // 复位成功后拉�?,作为flag

                        if (!init_mpu) begin            // 还没初始�??
                            mpu_init_start <= 1;
                            state <= INIT_MPU;
                        end else begin
                            mpu_read_start <= 1;
                            state <= MPU_CAPTURE;
                        end
                    end
                INIT_MPU:       // state = 1
                    begin
                        mpu_init_start <= 0;        // 初始化开始后拉低,作为flag

                        // 给足够的时间让mpu初始化,共用一个delay_cnt
                        if (delay_cnt == CONTROL_CYCLE) begin
                            delay_cnt <= 0;
                            state <= mpu_error ? SYS_ERROR : IDLE;
                            init_mpu <= mpu_error ? 0 : 1;
                        end else begin
                            delay_cnt <= delay_cnt + 1;
                        end
                    end
                MPU_CAPTURE:    // state = 3
                    begin
                        mpu_read_start <= 0;        // 开始后拉低,作为flag
                        if (mpu_read_done) begin
                            pitch_start <= 1;
                            state <= PITCH_CAL;
                        end
                    end
                PITCH_CAL:      // state = 3
                    begin
                        pitch_start <= 0;           // 开始后拉低,作为flag
                        if (pitch_done) begin
                            cur_pitch_acc <= crda_angle_pitch;
                            roll_start <= 1;
                            state <= ROLL_CAL;
                        end
                    end
                ROLL_CAL:       // state = 4
                    begin
                        roll_start <= 0;            // 开始后拉低,作为flag
                        if (roll_done) begin
                            cur_roll_acc <= crda_angle_roll;
                            filter_en <= 1;
                            state <= CMP_FILTER;
                        end
                    end
                CMP_FILTER:     // state = 5
                    begin
                        filter_en <= 0;         // 开始后拉低,作为flag
                        cal_error_en <= 1;
                        state <= CAL_ERROR;
                    end
                CAL_ERROR:      // state = 6
                    begin
                        cal_error_en <= 0;
                        cal_pid_en <= 1;
                        state <= CAL_PID;
                    end
                CAL_PID:        // state = 7
                    begin

                        cal_pid_en <= 0;
                        state <= PWM_OUT;
                    end
                PWM_OUT:        // state = 8
                    begin
                        duty_1_oe <= 1;
                        duty_2_oe <= 1;
                        duty_3_oe <= 1;
                        duty_4_oe <= 1;
                        state <= NEXT_CAPTURE;
                    end
                NEXT_CAPTURE:   // state = 9
                    begin
                        duty_1_oe <= 0;
                        duty_2_oe <= 0;
                        duty_3_oe <= 0;
                        duty_4_oe <= 0;
                        if (delay_cnt == CONTROL_CYCLE) begin //决定了控制频�?
                            delay_cnt <= 0;
                            mpu_read_start <= 1;
                            state <= MPU_CAPTURE;
                        end else begin
                            delay_cnt <= delay_cnt + 1;
                        end
                    end
                SYS_ERROR:      // state = a
                    begin
                        error_code <= 8'h1;
                        if (error_reset != 0) begin
                            error_code <= 0;
                            mpu_error_reset <= 1;
                            state <= IDLE;
                        end
                    end
                default: state <= IDLE;
            endcase
        end
    end

endmodule



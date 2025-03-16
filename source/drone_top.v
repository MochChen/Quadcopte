`include "bb_iic.sv"
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
    // rs232模块信号
    wire RxD_idle;
    wire RxD_endofpacket;
    wire RxD_data_ready;
    wire [7:0] RxD_data;
    // rs232模块实例化
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

    // I2C模块信号
    wire rst_n = ~reset;    // 转换为低有效
    reg mpu_init;
    wire mpu_init_done;
    reg mpu_transfer;
    wire mpu_data_avalid;
    wire [7:0] mpu_data;
    wire mpu_busy;
    // I2C模块实例化
    bb_iic #(
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
        .data_avalid(mpu_data_avalid),
        .data(mpu_data),
        .busy_now(mpu_busy)
    );

    // PWM模块信号
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

    // PID模块信号
    reg calc_pid_oe;
    reg [15:0] target_from_232, current_from_mpu;
    wire to_pwm_oe;
    wire [15:0] to_pwm;
    // PID模块实例化
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

    // MPU数据缓冲
    reg [15:0] mpu_data_packed;    // 完整数据（示例用Z轴加速度）
    reg [3:0] byte_counter;
    wire out_of_control;

    // 三段式状态机 //////////////////////////////////

    typedef enum reg [2:0] {
        IDLE, MPU_CAPTURE, PID_CONTROL, PWM_OUT, ERROR
    } state_t;
    reg [2:0] state, next_state;

    // 时序逻辑
    always @(posedge clk or posedge reset) begin
        if (reset) state <= IDLE;
        else state <= next_state;
    end

    // 组合逻辑
    always @(*) begin
        case (state)
            IDLE: next_state = mpu_init_done ? MPU_CAPTURE : IDLE;
            MPU_CAPTURE: next_state = (byte_counter == 6) ? PID_CONTROL : MPU_CAPTURE;
            PID_CONTROL: next_state = out_of_control ? ERROR : PWM_OUT;
            PWM_OUT: next_state =  MPU_CAPTURE;
            ERROR:;
            default: next_state = IDLE;
        endcase
    end

    // 算法处理 与 信号输出





    

endmodule
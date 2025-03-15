`include "bb_iic.sv"
`include "bb_pwm.sv"
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
    // rs232
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

    // I2C模块信号
    wire rst_n = ~reset;    // 转换为低有效
    reg mpu_init;
    wire init_done;
    reg mpu_transfer;
    wire mpu_data_avalid;
    wire [7:0] mpu_data;
    wire mpu_busy;

    // I2C模块实例化
    bb_iic #(
        .CLK_MAIN(50000000),
        .SCL_DIV(800000)
    ) i2c_inst (
        .clk(clk),
        .scl(scl),
        .sda(sda),
        .rst_n(rst_n),
        .mpu_init(mpu_init),
        .init_done(init_done),
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
    ) pwm_inst (
        .clk(clk),
        .rst(reset),
        .speed_in(pid_output),
        .speed_oe(pwm_update),
        .pwm_out(pwm_out),
        .busy(pwm_busy)
    );

    // MPU数据缓冲
    reg [15:0] mpu_data_packed;    // 完整数据（示例用Z轴加速度）
    reg [3:0] byte_counter;


    // PID ////////////////////////////////////////
    // 后续增加232控制参数功能 
    // PID参数
    parameter KP = 16'd10;
    parameter KI = 16'd2;
    parameter KD = 16'd5;

    reg [15:0] target_val, current_val;
    reg [15:0] integral, prev_error;

    // PID计算函数
    function [15:0] calc_pid;
        input [15:0] target, current;
        reg [15:0] error, p_term, i_term, d_term;
        begin
            error = target - current;
            p_term = KP * error;
            i_term = KI * error;
            d_term = KD * (error - prev_error);
            calc_pid = p_term + integral + d_term;
        end
    endfunction

    // 三段式状态机 //////////////////////////////////

    typedef enum reg [2:0] {
        IDLE, GET_MPU, CAL_ATTITUDE,
        CAL_PID, UPDATE_PWM, ERROR
    } state_t;

    typedef enum reg [2:0] {
        SUB_BALANCE, SUB_UP, SUB_DOWN, SUB_LEFT,
        SUB_RIGHT, SUB_FORWARD, SUB_BACKWARD
    } sub_state_t;

    reg [2:0] state, next_state;
    reg [2:0] sub_state;

    // 时序逻辑
    always @(posedge clk or posedge reset) begin
        if (reset) state <= IDLE;
        else state <= next_state;
    end

    // 组合逻辑
    always @(*) begin
        case (state)
            IDLE: next_state = init_done ? GET_MPU : IDLE;
            GET_MPU: next_state = (byte_counter == 6) ? CAL_ATTITUDE : GET_MPU;
            CAL_ATTITUDE: next_state = out_of_control ? ERROR : CAL_PID;
            CAL_PID: next_state =  UPDATE_PWM;
            UPDATE_PWM: next_state = max_acc_time ? GET_MPU : UPDATE_PWM;
            ERROR:
            default: next_state = IDLE;
        endcase
    end

endmodule
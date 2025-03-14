module drone_top (
    input clk,              // 50MHz主时钟
    input reset,            // 高有效复位
    input cmd_valid,        // 上位机指令有效
    input cmd_oe,           // 指令输出使能
    input [7:0] cmd_data,   // 上位机指令数据
    input out_of_control,   // 失控信号
    output wire scl,        // I2C时钟
    inout wire sda,         // I2C数据
    output wire pwm_out     // PWM输出（单电机示例）
);

    // I2C模块信号
    wire rst_n = ~reset;    // 转换为低有效
    wire mpu_data_avalid;
    wire [7:0] mpu_data_byte;
    wire mpu_busy;
    reg mpu_init;
    reg mpu_transfer;

    // PWM模块信号
    wire pwm_busy;
    reg [15:0] pid_output;
    reg pwm_update;

    // MPU数据缓冲
    reg [15:0] mpu_data;    // 完整数据（示例用Z轴加速度）
    reg [3:0] byte_counter;

    // 状态定义
    typedef enum reg [2:0] {
        IDLE,
        GET_MPU,
        CAL_ATTITUDE,
        CAL_PID,
        UPDATE_PWM,
        ERROR
    } state_t;

    typedef enum reg [2:0] {
        SUB_BALANCE,
        SUB_UP,
        SUB_DOWN,
        SUB_LEFT,
        SUB_RIGHT,
        SUB_FORWARD,
        SUB_BACKWARD
    } sub_state_t;

    reg [2:0] state, next_state;
    reg [2:0] sub_state;

    // PID参数
    parameter KP = 16'd10;
    parameter KI = 16'd2;
    parameter KD = 16'd5;

    // PID中间变量
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
        .mpu_transfer(mpu_transfer),
        .data_avalid(mpu_data_avalid),
        .data(mpu_data_byte),
        .busy_now(mpu_busy)
    );

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

    // 时序逻辑
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            integral <= 16'd0;
            prev_error <= 16'd0;
            pid_output <= 16'd0;
            pwm_update <= 1'b0;
            mpu_init <= 1'b0;
            mpu_transfer <= 1'b0;
            byte_counter <= 4'd0;
            mpu_data <= 16'd0;
        end
        else begin
            state <= next_state;
            // MPU数据累积
            if (state == GET_MPU && mpu_data_avalid) begin
                case (byte_counter)
                    4'd4: mpu_data[15:8] <= mpu_data_byte; // Z轴加速度高字节
                    4'd5: mpu_data[7:0] <= mpu_data_byte;  // Z轴加速度低字节
                endcase
                byte_counter <= byte_counter + 1;
            end
            // 更新PID状态
            if (state == CAL_PID) begin
                integral <= integral + (KI * (target_val - current_val));
                prev_error <= target_val - current_val;
            end
        end
    end

    // 组合逻辑
    always @(*) begin
        next_state = state;
        sub_state = SUB_BALANCE;
        target_val = 16'd0;
        current_val = mpu_data;
        pwm_update = 1'b0;
        mpu_init = 1'b0;
        mpu_transfer = 1'b0;

        case (state)
            IDLE: begin
                if (cmd_valid) begin
                    mpu_init = 1'b1; // 初始化MPU
                    next_state = GET_MPU;
                end
            end

            GET_MPU: begin
                if (!mpu_busy && byte_counter == 0) begin
                    mpu_transfer = 1'b1; // 开始连续读取
                end
                if (byte_counter >= 4'd6) begin
                    next_state = CAL_ATTITUDE;
                    byte_counter = 4'd0;
                end
            end

            CAL_ATTITUDE: begin
                if (out_of_control) next_state = ERROR;
                else next_state = CAL_PID;
            end

            CAL_PID: begin
                if (cmd_oe) begin
                    case (cmd_data[2:0])
                        3'd0: sub_state = SUB_BALANCE;
                        3'd1: begin
                            sub_state = SUB_UP;
                            target_val = 16'd100;
                        end
                        3'd2: begin
                            sub_state = SUB_DOWN;
                            target_val = 16'd50;
                        end
                        3'd3: begin
                            sub_state = SUB_LEFT;
                            target_val = 16'd10;
                        end
                        3'd4: begin
                            sub_state = SUB_RIGHT;
                            target_val = 16'd10;
                        end
                        3'd5: begin
                            sub_state = SUB_FORWARD;
                            target_val = 16'd15;
                        end
                        3'd6: begin
                            sub_state = SUB_BACKWARD;
                            target_val = 16'd15;
                        end
                        default: sub_state = SUB_BALANCE;
                    endcase
                end
                pid_output = (sub_state == SUB_BALANCE) ? calc_pid(16'd0, mpu_data) : calc_pid(target_val, current_val);
                next_state = UPDATE_PWM;
            end

            UPDATE_PWM: begin
                pwm_update = 1'b1;
                if (!pwm_busy) next_state = IDLE; // 等待PWM调整完成
            end

            ERROR: begin
                pid_output = 16'd0;
                if (!reset) next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

endmodule
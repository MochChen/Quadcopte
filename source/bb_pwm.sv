`include "async.v"


module bb_pwm #(
    parameter MAX_SPEED = 65536,
    parameter MIN_SPEED = 256, 
    parameter ACC = 2560,
    parameter DEAD_ZONE = ACC / 2,
    parameter STATE_WIDTH = 3
)(
    input clk, rst,
    input wire [15:0] speed_in,
    input wire speed_oe,
    output wire pwm_out,
    output wire busy
);

    // 参数定义
    localparam TOP_IDLE = 0, TOP_ACTIVE = 1;

    // 状态寄存器
    reg [STATE_WIDTH - 1 : 0] state, next_state;
    always @(posedge clk or posedge rst) begin
        if (rst) state <= TOP_IDLE;
        else state <= next_state;
    end

    // 下一状态逻辑（组合）
    always @(*) begin
        case (state)
            TOP_IDLE: next_state = speed_oe ? TOP_ACTIVE : TOP_IDLE;
            TOP_ACTIVE: begin
                if ((speed_reg <= pwm_reg + DEAD_ZONE) && (speed_reg >= pwm_reg - DEAD_ZONE))
                    next_state = TOP_IDLE;
                else
                    next_state = TOP_ACTIVE;
            end
            default: next_state = TOP_IDLE;
        endcase
    end

    // 计数器和 PWM 寄存器（时序）
    reg [15:0] cnt = 0;
    reg [15:0] speed_reg = MIN_SPEED;
    reg [15:0] pwm_reg = MIN_SPEED;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt <= 0;
            speed_reg <= MIN_SPEED;
            pwm_reg <= MIN_SPEED;
        end else begin
            cnt <= cnt + 256;  // 固定步进
            case (state)
                TOP_IDLE: begin
                    if (speed_oe) speed_reg <= speed_in;
                end
                TOP_ACTIVE: begin
                    if (cnt == 65280) begin  // 每周期调整一次
                        if (speed_reg > pwm_reg + DEAD_ZONE) begin
                            pwm_reg <= (pwm_reg + ACC > MAX_SPEED) ? MAX_SPEED : pwm_reg + ACC;
                        end 
                        else if (speed_reg < pwm_reg - DEAD_ZONE) begin
                            pwm_reg <= (pwm_reg < MIN_SPEED + ACC) ? MIN_SPEED : pwm_reg - ACC;
                        end
                    end
                end
            endcase
        end
    end

    // 输出逻辑（组合）
    assign pwm_out = (cnt >= pwm_reg) ? 1 : 0;
    assign busy = (state == TOP_ACTIVE);

endmodule
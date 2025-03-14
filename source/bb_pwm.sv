// 使用100Mhz 的clk时候,大约50us完成加速,也就是控制空窗期50us

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
    output busy
);

    // top state machine
    parameter TOP_IDLE = 0, TOP_ACTIVE = 1; // TOP_ERROR = 2;
    reg [STATE_WIDTH - 1 : 0] top_state, next_top_state;

    always @(posedge clk or posedge rst) begin
        if (rst) top_state <= TOP_IDLE;
        else top_state <= next_top_state;
    end

    always @(posedge clk) begin
        case (top_state)
            TOP_IDLE: next_top_state <= speed_oe ? TOP_ACTIVE : TOP_IDLE;
            TOP_ACTIVE: begin
                if ((speed_reg <= pwm_reg + DEAD_ZONE) && (speed_reg >= pwm_reg - DEAD_ZONE)) begin
                    next_top_state <= TOP_IDLE;
                end
                else begin
                    next_top_state <= TOP_ACTIVE;
                end
            end 
            // TOP_ERROR: next_top_state <= rst ? TOP_IDLE : TOP_ERROR;
            default: next_top_state <= TOP_IDLE;
        endcase
    end

    reg [15:0] cnt = 0;
    reg [15:0] speed_reg = MIN_SPEED;
    reg [15:0] pwm_reg = MIN_SPEED;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt <= 0;
            speed_reg <= MIN_SPEED;
            pwm_reg <= MIN_SPEED;
        end
        else begin
            cnt <= cnt + 256;
            case (top_state)        
                TOP_IDLE: if (speed_oe) speed_reg <= speed_in;
                TOP_ACTIVE: 
                    // if (speed_oe) speed_reg <= speed_in;
                    if (cnt == 65280) begin
                        if (speed_reg > pwm_reg + DEAD_ZONE) begin
                            pwm_reg <= (pwm_reg + ACC > MAX_SPEED) ? MAX_SPEED : pwm_reg + ACC;
                        end 
                        else if (speed_reg < pwm_reg - DEAD_ZONE) begin
                            pwm_reg <= (pwm_reg < MIN_SPEED + ACC) ? MIN_SPEED : pwm_reg - ACC;
                        end
                    end    
                default: next_top_state <= TOP_IDLE; 
            endcase
        end
    end

    assign pwm_out = (cnt >= pwm_reg) ? 1 : 0;
    assign busy = (top_state == TOP_ACTIVE);

endmodule
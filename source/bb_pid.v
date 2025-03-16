module bb_pid #(
    parameter KP = 16'd10,
    parameter KI = 16'd2,
    parameter KD = 16'd5
)(
    input clk, rst,

    input wire calc_pid_oe,
    input [15:0] target_from_232, current_from_mpu,

    output reg to_pwm_oe,
    output reg [15:0] to_pwm
);
    
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

    always @(posedge clk or posedge rst) 
        if (rst) begin
            integral <= 0;
            prev_error <= 0;
        end
        else
        begin
            if (calc_pid_oe) begin
                to_pwm <= calc_pid (target_from_232, current_from_mpu);
                to_pwm_oe <= 1;
            end
            else
            begin
                to_pwm_oe <= 0;
            end
        end

endmodule
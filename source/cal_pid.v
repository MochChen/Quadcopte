module cal_pid #(
    parameter Kp = 100,
    parameter Ki = 1,
    parameter Kd = 1
)(
    input wire clk,
    input wire rst_n,
    input wire cal_pid_en,
    input wire [23:0] PWM_base,
    input wire [23:0] pitch_error,
    input wire [23:0] roll_error,
    input wire [23:0] yaw_error,
    input wire [23:0] i_pitch_error,
    input wire [23:0] i_roll_error,
    input wire [23:0] i_yaw_error,
    input wire [23:0] d_pitch_error,
    input wire [23:0] d_roll_error,
    input wire [23:0] d_yaw_error,

    output reg [15:0] pwm_duty_1,
    output reg [15:0] pwm_duty_2,
    output reg [15:0] pwm_duty_3,
    output reg [15:0] pwm_duty_4

);
    reg [23:0] pre_pitch_error;
    reg [23:0] pre_roll_error;
    reg [23:0] pre_yaw_error;
    always @(posedge clk) begin
        if (!rst_n) begin
            pwm_duty_1 <= 0;
            pwm_duty_2 <= 0;
            pwm_duty_3 <= 0;
            pwm_duty_4 <= 0;
        end if (cal_pid_en) begin
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
    end

endmodule
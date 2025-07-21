module pid (
    input  wire clk,
    input  wire rst_n,

    input  wire cal_pid_en,
    input  wire signed [15:0] pwm_base,

    input  wire signed [15:0] pitch_error,
    input  wire signed [15:0] roll_error,
    input  wire signed [15:0] yaw_error,

    input  wire signed [15:0] i_pitch_error,
    input  wire signed [15:0] i_roll_error,
    input  wire signed [15:0] i_yaw_error,
    
    input  wire signed [15:0] d_pitch_error,
    input  wire signed [15:0] d_roll_error,
    input  wire signed [15:0] d_yaw_error,

    output reg [15:0] pwm_duty_1,
    output reg [15:0] pwm_duty_2,
    output reg [15:0] pwm_duty_3,
    output reg [15:0] pwm_duty_4,

    input  wire [7:0] KP,//必须小于128
    input  wire [7:0] KI,//必须小于128
    input  wire [7:0] KD//必须小于128
);

    wire signed [25:0] temp_pwm_base = pwm_base;

    // PID计算,temp最大值为:24bit值 * 3
    localparam integer P_SHIFT = 7;
    localparam integer I_SHIFT = 9;
    localparam integer D_SHIFT = 8;
    wire signed [8:0] KP_signed = {1'b0, KP};
    wire signed [8:0] KI_signed = {1'b0, KI};
    wire signed [8:0] KD_signed = {1'b0, KD};

    (* keep *)wire signed [25:0] temp_pitch = (KP_signed * pitch_error   >>> P_SHIFT) +
                                              (KI_signed * i_pitch_error >>> I_SHIFT) +
                                              (KD_signed * d_pitch_error >>> D_SHIFT);

    (* keep *)wire signed [25:0] temp_roll = (KP_signed * roll_error    >>> P_SHIFT) +
                                             (KI_signed * i_roll_error  >>> I_SHIFT) +
                                             (KD_signed * d_roll_error  >>> D_SHIFT);

    (* keep *)wire signed [25:0] temp_yaw = (KP_signed * yaw_error     >>> P_SHIFT) +
                                            (KI_signed * i_yaw_error   >>> I_SHIFT) +
                                            (KD_signed * d_yaw_error   >>> D_SHIFT);

    // 四路电机的 PID 加权和
    (* keep *)wire signed [28:0] sum_1 = temp_pwm_base + temp_pitch + temp_roll + temp_yaw;
    (* keep *)wire signed [28:0] sum_2 = temp_pwm_base + temp_pitch - temp_roll - temp_yaw;
    (* keep *)wire signed [28:0] sum_3 = temp_pwm_base - temp_pitch + temp_roll - temp_yaw;
    (* keep *)wire signed [28:0] sum_4 = temp_pwm_base - temp_pitch - temp_roll + temp_yaw;

    always @(posedge clk) begin
        if (!rst_n) begin
            pwm_duty_1 <= 0;
            pwm_duty_2 <= 0;
            pwm_duty_3 <= 0;
            pwm_duty_4 <= 0;
        end else if (cal_pid_en) begin
            pwm_duty_1 <= (sum_1 > 29'sd65535)  ? 16'd65535 :
                          (sum_1 < 29'sd0)      ? 16'd0 :
                           sum_1;

            pwm_duty_2 <= (sum_2 > 29'sd65535)  ? 16'd65535 :
                          (sum_2 < 29'sd0)      ? 16'd0 :
                           sum_2;

            pwm_duty_3 <= (sum_3 > 29'sd65535)  ? 16'd65535 :
                          (sum_3 < 29'sd0)      ? 16'd0 :
                           sum_3;

            pwm_duty_4 <= (sum_4 > 29'sd65535)  ? 16'd65535 :
                          (sum_4 < 29'sd0)      ? 16'd0 :
                           sum_4;
        end
    end
endmodule

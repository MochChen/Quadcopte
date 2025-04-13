module cal_error (
    input wire clk,
    input wire rst_n,
    input wire cal_error_en,
    input wire [23:0] tgt_pitch,
    input wire [23:0] tgt_roll,
    input wire [23:0] tgt_yaw,
    input wire [23:0] cur_pitch,
    input wire [23:0] cur_roll,
    input wire [23:0] cur_yaw,
    output reg [23:0] pitch_error,
    output reg [23:0] roll_error,
    output reg [23:0] yaw_error,
    output reg [23:0] i_pitch_error,
    output reg [23:0] i_roll_error,
    output reg [23:0] i_yaw_error,
    output reg [23:0] d_pitch_error,
    output reg [23:0] d_roll_error,
    output reg [23:0] d_yaw_error
);
    reg [23:0] pre_pitch_error;
    reg [23:0] pre_roll_error;
    reg [23:0] pre_yaw_error;
    always @(posedge clk) begin
        if (!rst_n) begin
            pitch_error <= 0;
            roll_error <= 0;
            yaw_error <= 0;
            i_pitch_error <= 0;
            i_roll_error <= 0;
            i_yaw_error <= 0;
            d_pitch_error <= 0;
            d_roll_error <= 0;
            d_yaw_error <= 0;
            pre_pitch_error <= 0;
            pre_roll_error <= 0;
            pre_yaw_error <= 0;
        end if (cal_error_en) begin
            // 误差
            pitch_error <= tgt_pitch - cur_pitch;
            roll_error <= tgt_roll - cur_roll;
            yaw_error <= tgt_yaw - cur_yaw;

            // 更新积分
            i_pitch_error <= i_pitch_error + pitch_error;   //可能会溢出
            i_roll_error <= i_roll_error + roll_error;
            i_yaw_error <= i_yaw_error + yaw_error;
            // 更新微分
            d_pitch_error <= (pitch_error - pre_pitch_error); // 这里应该是除以dt,暂时用0.9代替
            d_roll_error <= (roll_error - pre_roll_error);
            d_yaw_error <= (yaw_error - pre_yaw_error);

            // 保存当前误差为下一次使用
            pre_pitch_error <= pitch_error;
            pre_roll_error <= roll_error;
            pre_yaw_error <= yaw_error;
        end
    end

endmodule
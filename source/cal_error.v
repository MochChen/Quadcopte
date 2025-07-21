module cal_error (
    input wire clk,
    input wire rst_n,

    input wire cal_error_en,

    input wire signed [15:0] tgt_height,  // 目标高度,暂不处理
    input wire signed [15:0] tgt_pitch,
    input wire signed [15:0] tgt_roll,
    input wire signed [15:0] tgt_yaw,

    input wire signed [15:0] cur_pitch,
    input wire signed [15:0] cur_roll,
    input wire signed [15:0] cur_yaw,

    input wire signed [15:0] pitch_gyro,
    input wire signed [15:0] roll_gyro,
    input wire signed [15:0] yaw_gyro,

    output reg signed [15:0] pitch_error,
    output reg signed [15:0] roll_error,
    output reg signed [15:0] yaw_error,

    output reg signed [15:0] i_pitch_error,
    output reg signed [15:0] i_roll_error,
    output reg signed [15:0] i_yaw_error,

    output reg signed [15:0] d_pitch_error,
    output reg signed [15:0] d_roll_error,
    output reg signed [15:0] d_yaw_error
);

    reg signed [15:0] pre_pitch_error;
    reg signed [15:0] pre_roll_error;
    reg signed [15:0] pre_yaw_error;

    //比例中间变量
    wire signed [16:0] pitch_sum = tgt_pitch - cur_pitch;
    wire signed [16:0] roll_sum = tgt_roll - cur_roll;
    wire signed [16:0] yaw_sum = tgt_yaw - cur_yaw;

    // 积分中间变量,缩放处理,在KI可以放大回来
    wire signed [17:0] i_pitch_sum = i_pitch_error + pitch_sum;
    wire signed [17:0] i_roll_sum = i_roll_error + roll_sum;
    wire signed [17:0] i_yaw_sum = i_yaw_error + yaw_sum;

    // 微分

    always @(posedge clk or negedge rst_n) begin
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

        end else if (cal_error_en) begin
            // 当前误差
            pitch_error <= (pitch_sum > 17'sd32767)  ? 16'sd32767 :
                           (pitch_sum < -17'sd32768) ? -16'sd32768 :
                            pitch_sum[15:0];
            roll_error  <= (roll_sum > 17'sd32767)  ? 16'sd32767 :
                           (roll_sum < -17'sd32768) ? -16'sd32768 :
                            roll_sum[15:0];
            yaw_error   <= (yaw_sum > 17'sd32767)  ? 16'sd32767 :
                           (yaw_sum < -17'sd32768) ? -16'sd32768 :
                            yaw_sum[15:0];

            // 积分
            i_pitch_error <= (i_pitch_sum > 18'sd32767)  ? 16'sd32767 :
                             (i_pitch_sum < -18'sd32768) ? -16'sd32768 :
                              i_pitch_sum[15:0];
            i_roll_error  <= (i_roll_sum > 18'sd32767)  ? 16'sd32767 :
                             (i_roll_sum < -18'sd32768) ? -16'sd32768 :
                              i_roll_sum[15:0];
            i_yaw_error   <= (i_yaw_sum > 18'sd32767)  ? 16'sd32767 :
                             (i_yaw_sum < -18'sd32768) ? -16'sd32768 :
                              i_yaw_sum[15:0];

            // 微分
            d_pitch_error <= -pitch_gyro;
            d_roll_error  <= -roll_gyro;
            d_yaw_error   <= -yaw_gyro;

            // 更新历史值
            pre_pitch_error <= pitch_error;
            pre_roll_error  <= roll_error;
            pre_yaw_error   <= yaw_error;
        end
    end

endmodule

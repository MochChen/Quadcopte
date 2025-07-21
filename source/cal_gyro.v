module cal_gyro (
    input wire clk,
    input wire rst_n,

    input wire cal_gyro_oe,

    input wire signed [15:0] pitch_gyro,
    input wire signed [15:0] roll_gyro,
    input wire signed [15:0] yaw_gyro,

    output reg signed [15:0] cur_pitch_gyro,
    output reg signed [15:0] cur_roll_gyro,
    output reg signed [15:0] cur_yaw_gyro

);

    reg signed [15:0] in_pitch_gyro;
    reg signed [15:0] in_roll_gyro;
    reg signed [15:0] in_yaw_gyro;

    wire signed [31:0] delta_pitch_gyro_q16 = in_pitch_gyro * 1311;
    wire signed [31:0] delta_roll_gyro_q16  = in_roll_gyro * 1311;
    wire signed [31:0] delta_yaw_gyro_q16   = in_yaw_gyro * 1311;

    wire signed [15:0] delta_pitch_gyro = delta_pitch_gyro_q16 >>> 16;
    wire signed [15:0] delta_roll_gyro  = delta_roll_gyro_q16  >>> 16;
    wire signed [15:0] delta_yaw_gyro   = delta_yaw_gyro_q16   >>> 16;

    wire signed [16:0] sum_pitch_gyro = cur_pitch_gyro + delta_pitch_gyro + 1;
    wire signed [16:0] sum_roll_gyro  = cur_roll_gyro + delta_roll_gyro + 1;
    wire signed [16:0] sum_yaw_gyro   = cur_yaw_gyro + delta_yaw_gyro + 1;


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_pitch_gyro <= 0;
            in_roll_gyro  <= 0;
            in_yaw_gyro   <= 0;

            cur_pitch_gyro <= 0;
            cur_roll_gyro  <= 0;
            cur_yaw_gyro   <= 0;

        end else begin
            if (cal_gyro_oe) begin
                in_pitch_gyro <= pitch_gyro;
                in_roll_gyro  <= roll_gyro;
                in_yaw_gyro   <= yaw_gyro;

                cur_pitch_gyro <= (sum_pitch_gyro > 17'sd32767)  ? 16'sd32767 :
                                  (sum_pitch_gyro < -17'sd32768) ? -16'sd32768 :
                                   sum_pitch_gyro[15:0];
                cur_roll_gyro  <= (sum_roll_gyro > 17'sd32767)  ? 16'sd32767 :
                                  (sum_roll_gyro < -17'sd32768) ? -16'sd32768 :
                                   sum_roll_gyro[15:0];
                cur_yaw_gyro   <= (sum_yaw_gyro > 17'sd32767)  ? 16'sd32767 :
                                  (sum_yaw_gyro < -17'sd32768) ? -16'sd32768 :
                                   sum_yaw_gyro[15:0];
                
            end
        end
    end


endmodule
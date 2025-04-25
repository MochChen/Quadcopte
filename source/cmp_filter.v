module cmp_filter #(
    parameter ALPHA = 99
)(
    input wire clk,
    input wire rst_n,
    input wire cmp_filter_en,
    input wire [23:0] cur_pitch_gyro,
    input wire [23:0] cur_roll_gyro,
    input wire [23:0] cur_yaw_gyro,
    input wire [23:0] cur_pitch_acc,
    input wire [23:0] cur_roll_acc,

    output reg signed [23:0] cur_pitch,
    output reg signed [23:0] cur_roll,
    output reg signed [23:0] cur_yaw
);

    wire signed [40:0] pitch_temp; // 中间结果，假设为40位,cur_pitch_gyro实际数值只有16bit大小
    wire signed [40:0] roll_temp;
    assign pitch_temp = (ALPHA * $signed(cur_pitch_gyro) + (100 - ALPHA) * $signed(cur_pitch_acc)) * 15'sd328;
    assign roll_temp = (ALPHA * $signed(cur_roll_gyro) + (100 - ALPHA) * $signed(cur_roll_acc)) * 15'sd328;

    always @(posedge clk) begin
        if (!rst_n) begin
            cur_pitch <= 0;
            cur_roll <= 0;
            cur_yaw <= 0;
        end else if (cmp_filter_en) begin
            cur_pitch <= (pitch_temp >>> 15); // 乘以0.01并右移恢复精度
            cur_roll  <= (roll_temp >>> 15);
            cur_yaw   <= cur_yaw_gyro;
        end
    end


endmodule
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

    output reg [23:0] cur_pitch,
    output reg [23:0] cur_roll,
    output reg [23:0] cur_yaw
);
    
    always @(posedge clk) begin
        if (!rst_n) begin
            cur_pitch <= 0;
            cur_roll <= 0;
            cur_yaw <= 0;
        end if (cmp_filter_en) begin
            cur_pitch <= (ALPHA * cur_pitch_gyro + (100 - ALPHA) * cur_pitch_acc) / 100;
            cur_roll  <= (ALPHA * cur_roll_gyro  + (100 - ALPHA) * cur_roll_acc) / 100;
            cur_yaw   <= cur_yaw_gyro;
        end
    end


endmodule
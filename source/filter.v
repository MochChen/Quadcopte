module filter (
    input wire clk,
    input wire rst_n,
    input wire filter_en,

    input wire signed [15:0] cur_pitch_gyro,
    input wire signed [15:0] cur_roll_gyro,
    input wire signed [15:0] cur_yaw_gyro,

    input wire signed [15:0] cur_pitch_acc,
    input wire signed [15:0] cur_roll_acc,

    output reg signed [15:0] cur_pitch,
    output reg signed [15:0] cur_roll,
    output reg signed [15:0] cur_yaw,

    input wire signed [7:0] alpha  // 0-100
);

    // Q2.14: 0.01 ≈ 2^14 * 0.01 = 164
    wire signed [7:0] alpha_complement = 100 - alpha;

    wire signed [63:0] pitch_temp = (alpha * cur_pitch_gyro + alpha_complement * cur_pitch_acc) * 14'sd164;
    wire signed [63:0] roll_temp  = (alpha * cur_roll_gyro  + alpha_complement * cur_roll_acc)  * 14'sd164;
    wire signed [63:0] pitch_real = (pitch_temp >>> 14);
    wire signed [63:0] roll_real = (roll_temp >>> 14);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_pitch <= 0;
            cur_roll <= 0;
            cur_yaw <= 0;

        end else if (filter_en) begin
            cur_pitch <= pitch_real[15:0];
            cur_roll  <= roll_real[15:0];
            cur_yaw   <= 0;

        end
    end

endmodule

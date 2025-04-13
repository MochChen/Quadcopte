module cal_gyro #(
    parameter dt = 1
)(
    input wire clk,
    input wire rst_n,

    input wire cal_gyro_en,
    input wire [7:0] mpu_data_packed_8,
    input wire [7:0] mpu_data_packed_9,
    input wire [7:0] mpu_data_packed_10,
    input wire [7:0] mpu_data_packed_11,
    input wire [7:0] mpu_data_packed_12,
    input wire [7:0] mpu_data_packed_13,
    output  reg [23:0] cur_pitch_gyro,
    output  reg [23:0] cur_roll_gyro,
    output  reg [23:0] cur_yaw_gyro
);
    always @(posedge clk) begin
        if (!rst_n) begin
            cur_pitch_gyro <= 0;
            cur_roll_gyro <= 0;
            cur_yaw_gyro <= 0;
        end if (cal_gyro_en) begin
            cur_pitch_gyro <= cur_pitch_gyro + dt * $signed({mpu_data_packed_8, mpu_data_packed_9});
            cur_roll_gyro <= cur_roll_gyro + dt * $signed({mpu_data_packed_10, mpu_data_packed_11});
            cur_yaw_gyro <= cur_yaw_gyro + dt * $signed({mpu_data_packed_12, mpu_data_packed_13});
        end
    end
endmodule
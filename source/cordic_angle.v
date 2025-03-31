`include "cordic.v"

module cordic_angle #(
    parameter integer ITERATIONS = 12  // CORDIC 迭代次数
)(
    input clk,               // 时钟
    input rst_n,             // 复位，低有效
    input signed [15:0] x_in,  // X 轴加速度
    input signed [15:0] y_in,  // Y 轴加速度
    input signed [15:0] z_in,  // Z 轴加速度

    input start,
    output reg done,
    output reg signed [15:0] pitch,  
    output reg signed [15:0] roll    
);

    parameter reg [2:0] IDLE = 3'h0,
                        SQRT  = 3'h1,
                        DELAY = 3'h2,
                        ATAN2  = 3'h3, 
                        DONE = 3'h4;
    reg [2:0] state = IDLE;
    
    reg signed [15:0] x_p, y_p, x_r, y_r;
    reg start_p = 0, start_r = 0;
    wire done_p, done_r;
    wire signed [16:0] angle_p, angle_r;
    wire signed [15:0] magnitude_p, magnitude_r;

    // CORDIC 计算 Pitch
    cordic #(.ITERATIONS(12)) pitch_dut (
        .clk(clk),
        .rst_n(rst_n),
        .x_in(y_p),
        .y_in(x_p),
        .start(start_p),
        .done(done_p),
        .angle_out(angle_p),
        .magnitude_out(magnitude_p)
    );

    // CORDIC 计算 Roll
    cordic #(.ITERATIONS(12)) roll_dut (
        .clk(clk),
        .rst_n(rst_n),
        .x_in(y_r),
        .y_in(x_r),
        .start(start_r),
        .done(done_r),
        .angle_out(angle_r),
        .magnitude_out(magnitude_r)
    );

    // 状态机
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin  // 复位逻辑修正
            x_p <= 0;
            y_p <= 0;
            x_r <= 0;
            y_r <= 0;
            start_p <= 0;
            start_r <= 0;
            state <= IDLE;
            done <= 0;

            pitch <= 0;
            roll <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    done <= 0; //  防止 done 信号毛刺
                    if (start) begin
                        state <= SQRT;
                        // pitch = atan2(ax, sqrt(ay * ay + az * az)); 
                        // roll  = atan2(ay, sqrt(ax * ax + az * az)); 
                        x_p <= y_in;
                        y_p <= z_in;
                        start_p <= 1;
                        x_r <= x_in;
                        y_r <= z_in;
                        start_r <= 1;
                    end
                end
                SQRT: begin
                    start_p <= 0;
                    start_r <= 0;
                    if (done_p && done_r) begin 
                        state <= DELAY;
                        // pitch = atan2(ax, sqrt(ay * ay + az * az)); 
                        // roll  = atan2(ay, sqrt(ax * ax + az * az)); 
                        x_p <= x_in;
                        y_p <= magnitude_p;
                        x_r <= y_in;  // 负号计算修正,用~x_in + 1取代-x_in
                        y_r <= magnitude_r;
                    end
                end
                DELAY: begin
                    state <= ATAN2;
                    start_p <= 1;
                    start_r <= 1;
                end
                ATAN2: begin
                    start_p <= 0;
                    start_r <= 0;
                    if (done_p && done_r) begin 
                        state <= DONE;
                        pitch <= angle_p;
                        roll <= angle_r;
                        done <= 1;
                    end
                end
                DONE: begin
                    if (!start) begin
                        state <= IDLE; // 只有在 start 释放后才回到 IDLE
                        x_p <= 0;
                        y_p <= 0;
                        x_r <= 0;
                        y_r <= 0;
                        start_p <= 0;
                        start_r <= 0;
                        done <= 0;
                        pitch <= 0;
                        roll <= 0;
                    end 
                end
            endcase
        end
    end

endmodule

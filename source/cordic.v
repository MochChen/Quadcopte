module cordic #(
    parameter integer ITERATIONS = 12  // 增加迭代次数
)(
    input clk,               // 时钟
    input rst_n,             // 复位，低有效
    input signed [15:0] x_in,  // 输入向量X分量 in = +-32768
    input signed [15:0] y_in,  // 输入向量Y分量 in = +-32768
    input start,
    output reg done,
    output reg signed [16:0] angle_out,     // 16位角度输出
    output reg signed [15:0] magnitude_out  // 输出模 sqrt(ax^2 + ay^2)
);

    // CORDIC角度表 (atan(2^-i))，缩放因子 16384 表示 90°
    reg signed [19:0] atan_table [0:15];
    initial begin
        atan_table[0]  = 20'd8192;  // atan(2^0)  = 45°
        atan_table[1]  = 20'd4832;  // atan(2^-1) = 26.565°
        atan_table[2]  = 20'd2555;  // atan(2^-2) = 14.036°
        atan_table[3]  = 20'd1297;  // atan(2^-3) = 7.125°
        atan_table[4]  = 20'd652;   // atan(2^-4) = 3.576°
        atan_table[5]  = 20'd326;   // atan(2^-5) = 1.790°
        atan_table[6]  = 20'd163;   // atan(2^-6) = 0.895°
        atan_table[7]  = 20'd81;    // atan(2^-7) = 0.448°
        atan_table[8]  = 20'd41;    // atan(2^-8) = 0.224°
        atan_table[9]  = 20'd20;    // atan(2^-9) = 0.112°
        atan_table[10] = 20'd10;    // atan(2^-10)= 0.056°
        atan_table[11] = 20'd5;     // atan(2^-11)= 0.028°
        atan_table[12] = 20'd3;     // atan(2^-12)= 0.014°
        atan_table[13] = 20'd1;     // atan(2^-13)= 0.007°
        atan_table[14] = 20'd1;     // atan(2^-14)= 0.0035°
        atan_table[15] = 20'd0;     // atan(2^-15)= 0.0017°
    end

    // 寄存器定义
    reg signed [16:0] x, y; // 注意:atan2(y, x)
    reg signed [15:0] z;  // 16位角度寄存器
    reg [3:0] i;          // 迭代计数器
    reg [1:0] state = 2'h0;
    reg zero_input;  // 标志位，表示输入向量是否为零
    reg neg_x;

    // CORDIC增益倒数 (1/K ≈ 0.607253 * 2^14 ≈ 9949)
    localparam signed [15:0] GAIN_INV = 16'd9949;
    wire signed [31:0] Mul_Cache = x * GAIN_INV;

    always @(posedge clk or negedge rst_n) 
    begin
        if (!rst_n) begin
            x <= 0;
            y <= 0;
            z <= 0;
            angle_out <= 0;
            magnitude_out <= 0;
            done <= 0;
            state <= 0;
            i <= 0;
        end 
        else begin
            case (state)
                0: begin
                    if (start) begin
                        state <= 1;
                        i <= 0;
                        zero_input <= (x_in == 0 && y_in == 0); // 检测零输入
                        neg_x <= (x_in < 0);// 检测第2、3象限
                        x <= (x_in < 0) ? -x_in : x_in;
                        y <= y_in;
                        z <= 0;  // 初始角度设为 0
                    end
                end
                1: begin
                    if (zero_input && i == ITERATIONS) begin  // 如果输入为零，空占用ITERATIONS后输出
                        angle_out <= 0;    // 角度定义为0
                        magnitude_out <= 0;
                        state <= 2;
                        done <= 1;
                    end
                    else if (i < ITERATIONS) begin
                        if (y >= 0) begin
                            x <= x + (y >>> i); 
                            y <= y - (x >>> i);
                            z <= z + atan_table[i];
                        end 
                        else begin
                            x <= x - (y >>> i);
                            y <= y + (x >>> i);
                            z <= z - atan_table[i];
                        end
                        i <= i + 1;
                    end 
                    else begin
                        // **处理第2、3象限的情况**
                        // 如果 x_in < 0，需修正角度 ±180°
                        // if (x_in < 0) begin
                        //     angle_out <= z + (y_in >= 0 ? 16'd16384 : -16'd16384);  // ±180°
                        // end else begin
                        //     angle_out <= z;
                        // end

                        // 如果 x_in < 0，需修正角度 ±180°
                        if (neg_x) begin
                            angle_out <= -z;  // ±180°
                        end else begin
                            angle_out <= z;
                        end

                        //angle_out <= z;
                        magnitude_out <= Mul_Cache >>> 14;
                        state <= 2;
                        done <= 1;
                    end
                end
                2,3: begin
                    x <= 0;
                    y <= 0;
                    z <= 0;
                    angle_out <= 0;
                    magnitude_out <= 0;
                    done <= 0;
                    state <= 0;
                    i <= 0;
                    zero_input <= 0;
                end
                default: state <= 0;
            endcase
        end
    end
endmodule

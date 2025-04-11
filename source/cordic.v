// 输出的角度只会在正负90度之间,超过90度会作为负数处理:比如135度等于-45度

module cordic #(
    parameter integer ITERATIONS = 14  // 迭代次数
)(
    input clk,               
    input rst_n,             // 复位，低有效
    input signed [23:0] x,  // 输入向量X分量 in = +-32768, 需要封装层24bit
    input signed [23:0] y,  // 输入向量Y分量 in = +-32768, 需要封装层24bit
    input wire crd_start,
    output reg crd_done,
    output reg signed [23:0] crd_angle = 0,     // 16位角度输出
    output reg signed [23:0] crd_magnitude = 0 // 输出模 sqrt(ax^2 + ay^2)
);

    // 寄存器定义
    parameter   IDLE     = 3'h0,
                ROTATION = 3'h1,
                STOP     = 3'h2;
    reg [2:0] state = IDLE;
    reg signed [23:0] x_reg;
    reg signed [23:0] y_reg;
    reg signed [23:0] z_reg; //17bit 防止溢出
    reg [3:0] i = 0;
    reg [40:0] temp = 0;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            x_reg <= 0;
            y_reg <= 0;
            z_reg <= 0;
            i <= 0;
            crd_done <= 0;
        end else begin
            case (state)
                IDLE: 
                    begin
                        x_reg <= 0;
                        y_reg <= 0;
                        z_reg <= 0;
                        i <= 0;
                        crd_done <= 0;
                        if (crd_start) begin
                            if (x == 0 && y == 0) begin
                                crd_angle <= 0;
                                crd_magnitude <= 0;
                                crd_done <= 1;
                                state <= IDLE;
                            end else begin
                                state <= ROTATION;
                                x_reg <= (x > 0) ? x : -x;
                                y_reg <= (x > 0) ? y : -y;
                                z_reg <= 0;
                            end
                        end
                    end
                ROTATION: 
                    begin
                        if (i < ITERATIONS) begin
                            if (y_reg >= 0) begin
                                x_reg <= x_reg + (y_reg >>> i); 
                                y_reg <= y_reg - (x_reg >>> i);
                                z_reg <= z_reg + atan_table[i];
                            end else begin
                                x_reg <= x_reg - (y_reg >>> i);
                                y_reg <= y_reg + (x_reg >>> i);
                                z_reg <= z_reg - atan_table[i];
                            end
                            i <= i + 1;
                        end else begin
                            state <= STOP;
                            crd_angle <= z_reg;
                            temp <= (x_reg * 16'sd9949);//记下当前值 和 增益补偿(约等于 crd_magnitude * 0.607253)
                        end
                    end
                    STOP:
                        begin
                            crd_magnitude <= (temp >> 14);
                            state <= IDLE;
                            crd_done <= 1;
                        end
                default: state <= IDLE;
            endcase
        end
    end

    // CORDIC角度表 (atan(2^-i))，缩放因子 11790 表示 90°
    reg signed [23:0] atan_table [0:15];  // 17-bit signed

    initial begin
        atan_table[0]  = 24'sd5895;  // atan(2^0)  = 45.000° × 131 = 5895
        atan_table[1]  = 24'sd3480;  // atan(2^-1) = 26.565° × 131 = 3480
        atan_table[2]  = 24'sd1839;  // atan(2^-2) = 14.036° × 131 = 1839
        atan_table[3]  = 24'sd933;   // atan(2^-3) = 7.125°  × 131 = 933
        atan_table[4]  = 24'sd468;   // atan(2^-4) = 3.576°  × 131 = 468
        atan_table[5]  = 24'sd234;   // atan(2^-5) = 1.790°  × 131 = 234
        atan_table[6]  = 24'sd117;   // atan(2^-6) = 0.895°  × 131 = 117
        atan_table[7]  = 24'sd59;    // atan(2^-7) = 0.448°  × 131 = 59
        atan_table[8]  = 24'sd29;    // atan(2^-8) = 0.224°  × 131 = 29
        atan_table[9]  = 24'sd15;    // atan(2^-9) = 0.112°  × 131 = 15
        atan_table[10] = 24'sd7;     // atan(2^-10)= 0.056°  × 131 = 7
        atan_table[11] = 24'sd4;     // atan(2^-11)= 0.028°  × 131 = 4
        atan_table[12] = 24'sd2;     // atan(2^-12)= 0.014°  × 131 = 2
        atan_table[13] = 24'sd1;     // atan(2^-13)= 0.007°  × 131 = 1
        atan_table[14] = 24'sd0;     // atan(2^-14)= 0.0035° × 131 = 0.46 ≈ 0
        atan_table[15] = 24'sd0;     // atan(2^-15)= 0.0017° × 131 = 0.22 ≈ 0
    end


endmodule




    
    // // CORDIC角度表 (atan(2^-i))，缩放因子 16384 表示 90°
    // reg signed [23:0] atan_table [0:15];  // 17-bit signed

    // initial begin
    //     atan_table[0]  = 24'sd8192;  // atan(2^0)  = 45°
    //     atan_table[1]  = 24'sd4832;  // atan(2^-1) = 26.565°
    //     atan_table[2]  = 24'sd2555;  // atan(2^-2) = 14.036°
    //     atan_table[3]  = 24'sd1297;  // atan(2^-3) = 7.125°
    //     atan_table[4]  = 24'sd652;   // atan(2^-4) = 3.576°
    //     atan_table[5]  = 24'sd326;   // atan(2^-5) = 1.790°
    //     atan_table[6]  = 24'sd163;   // atan(2^-6) = 0.895°
    //     atan_table[7]  = 24'sd81;    // atan(2^-7) = 0.448°
    //     atan_table[8]  = 24'sd41;    // atan(2^-8) = 0.224°
    //     atan_table[9]  = 24'sd20;    // atan(2^-9) = 0.112°
    //     atan_table[10] = 24'sd10;    // atan(2^-10)= 0.056°
    //     atan_table[11] = 24'sd5;     // atan(2^-11)= 0.028°
    //     atan_table[12] = 24'sd3;     // atan(2^-12)= 0.014°
    //     atan_table[13] = 24'sd1;     // atan(2^-13)= 0.007°
    //     atan_table[14] = 24'sd1;     // atan(2^-14)= 0.0035°
    //     atan_table[15] = 24'sd0;     // atan(2^-15)= 0.0017°
    // end
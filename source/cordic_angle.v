// `include "cordic.v"
// pitch = atan2(ax, sqrt(ay * ay + az * az));
// roll  = atan2(ay, sqrt(ax * ax + az * az));

module cordic_angle #(
    parameter integer ITERATIONS = 14  // CORDIC 迭代次数
)(
    input clk,
    input rst_n,
    input signed [15:0] x,
    input signed [15:0] y,
    input signed [15:0] z,

    input wire cdra_start,
    output reg cdra_done,
    output reg signed [15:0] crda_angle
);

    localparam  IDLE        = 3'h0,
                CRD_START   = 3'h1,
                CAL_MGNI    = 3'h2,
                STOP        = 3'h3;
    reg [2:0] state;
    
    reg signed [23:0] x_reg;
    reg signed [23:0] y_reg;
    reg signed [23:0] z_reg;
    reg mgni = 0;

    reg crd_start;
    reg signed [23:0] crd_x;
    reg signed [23:0] crd_y;
    wire crd_done;
    wire signed [23:0] crd_angle;
    wire signed [23:0] crd_magnitude;
    reg signed [23:0] crd_magnitude_reg;// = crd_magnitude * 0.607253 = (crd_magnitude  * 0.607253 * 2^14 ) >>> 14

    cordic #(
        .ITERATIONS(14)
    ) cordic_angle_calc (
        .clk(clk),
        .rst_n(rst_n),
        .x(crd_x),
        .y(crd_y),
        .crd_start(crd_start),
        .crd_done(crd_done),
        .crd_angle(crd_angle),
        .crd_magnitude(crd_magnitude)
    );
    
    // 状态机
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin  
            cdra_done <= 0;
            crda_angle <= 0;

            x_reg <= 0;
            y_reg <= 0;
            z_reg <= 0;
            mgni <= 0;

            crd_start <= 0;
            crd_x <= 0;
            crd_y <= 0;
            crd_magnitude_reg <= 0;

            state <= IDLE;
        end else begin
            case (state)
                IDLE: 
                    begin
                        if (cdra_start) begin
                            x_reg <= {{8{x[15]}}, x}; //转换成24bit
                            y_reg <= {{8{y[15]}}, y};
                            z_reg <= {{8{z[15]}}, z};
                            state <= CRD_START;
                        end
                    end
                CRD_START:
                    begin
                        state <= CAL_MGNI;
                        if (mgni == 0) begin //第一次计算 模长
                            crd_start <= 1;
                            crd_x <= y_reg;
                            crd_y <= z_reg;
                        end else begin      //第二次计算 角度,CORDIC是atan2(y, x)，不是atan2(x, y)！
                            crd_start <= 1;
                            crd_x <= crd_magnitude_reg;
                            crd_y <= x_reg;
                        end
                    end
                CAL_MGNI: 
                    begin
                        crd_start <= 0;
                        if (crd_done) begin 
                            if (mgni == 0) begin
                                state <= CRD_START; // 再次计算
                                mgni <= 1;
                                crd_magnitude_reg <= crd_magnitude; //记下当前值 和 增益补偿(约等于 crd_magnitude * 0.607253)
                            end else begin
                                state <= STOP; // 结束
                                cdra_done <= 1;
                                crda_angle <= $signed(crd_angle[15:0]); // 23bit转成16bit,见下方说明
                                /************************************************************\
                                说明:
                                -   5895表示45.000°,缩放为131,和陀螺仪保持一致缩放倍数,直接加减;
                                -   最大值在cordic中限制了90度,也就是说crd_angle的最大值是正负11790,
                                    小于2的15次方正负32768,直接取低16位,不会影响精度.
                                \************************************************************/
                            end
                        end
                    end
                STOP: begin
                    state <= IDLE;
                    cdra_done <= 0;
                end
                default: state <= IDLE;
            endcase
        end
    end

endmodule

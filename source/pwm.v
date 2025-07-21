module pwm (
    input wire clk,
    input wire rst_n,
    
    input wire [15:0] speed_in,
    input wire speed_oe,
    output wire pwm,

    input wire [23:0] max_speed, //95_000
    input wire [23:0] min_speed  //50_000
);

    reg [16:0] speed_reg; 
    wire [23:0] sum_speed = speed_in + min_speed;
    reg [15:0] init_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            speed_reg <= 0;
            init_cnt <= 0;
            
        end else begin
            if (speed_oe) begin
                if (init_cnt == 1000) begin //前面800个mpu忽略
                    speed_reg <= (sum_speed > max_speed) ? max_speed : sum_speed;
                end else begin
                    init_cnt <= init_cnt + 1;
                    speed_reg <= min_speed;
                end     
            end
        end
    end

    // PWM周期计数器
    reg [19:0] cnt;        // 2ms计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 0;

        end else begin
            cnt <= (cnt == 100000) ?  0 : cnt + 16'd1;
        end
    end

    // 输出PWM波形
    assign pwm = (cnt < speed_reg) ? 1'b1 : 1'b0;

endmodule

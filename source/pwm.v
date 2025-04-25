module pwm #(
    parameter MAX_SPEED = 65536,
    parameter MIN_SPEED = 256
)(
    input clk,
    input rst_n,
    input wire [15:0] speed_in,
    input wire speed_oe,
    output wire pwm,
    output reg busy
);

    reg [15:0] speed_reg;
    reg [15:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0;
            cnt <= 0;
            speed_reg <= 0;
        end else begin
            if (speed_oe) begin
                speed_reg <= speed_in;
            end
            cnt <= cnt + 1;
        end
    end

    assign pwm = (cnt < speed_reg) ? 1 : 0;


endmodule
module mpu_mid (
    input clk,
    input rst_n,

    input init_start,
    input read_start,

    output reg en_start,  
    output reg rd_now, 

    output reg [15:0] n,
    output reg [15:0] m,
    //output [15:0] data_packed
    output [127:0] data_packed  // 16 × 8 = 128 位

);
    
    reg [7:0] data_0, data_1;
    assign data_packed = {112'b0, data_1, data_0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_0 <= 0;
            data_1 <= 0;
            en_start <= 0;
            rd_now <= 0;
            n <= 0;
            m <= 0;
            $display("  复位 [ok]");
        end else begin
            if (init_start) begin
                en_start <= 1;
                rd_now <= 0;
                n <= 1; // 写两字节
                m <= 0; // 不作用
                data_0 <= 8'h6B;
                data_1 <= 8'h00;
                $display("  初始化 [start...]");
            end else if (read_start) begin
                en_start <= 1;
                rd_now <= 1;
                n <= 0; // 写一字节
                m <= 13;
                data_0 <= 8'h3B;
                data_1 <= 8'h00;
                $display("  读 [start...]");
            end else begin
                en_start <= 0;  // 只在两者都没有时清零
            end
        end
    end

endmodule
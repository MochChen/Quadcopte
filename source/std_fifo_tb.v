`timescale 1ns/1ps
// `include "std_fifo.v"

module std_fifo_tb;

    reg rd_clk = 0;
    reg wr_clk = 0;
    always #7 rd_clk = ~rd_clk;  // 约71MHz
    always #5 wr_clk = ~wr_clk;  // 100MHz

    reg rd_rst_n = 0;
    reg wr_en = 0;
    reg rd_en = 0;
    reg [7:0] wr_data = 0;
    wire [7:0] rd_data;
    wire full, empty;

    std_fifo uut (
        .rd_rst_n(rd_rst_n),
        .rd_clk(rd_clk),
        .rd_en(rd_en),
        .empty(empty),
        .rd_data(rd_data),
        .wr_clk(wr_clk),
        .wr_en(wr_en),
        .full(full),
        .wr_data(wr_data)
    );

    initial begin

        // Reset
        #20 rd_rst_n = 1;

        #50

        // 写入数据
        repeat(35) begin
            @(posedge wr_clk);
            if (!full) begin
                wr_en = 1;
                wr_data = wr_data + 1;
            end
        end
        wr_en = 0;

        // 延迟后开始读取
        #100;

        repeat(35) begin
            @(posedge rd_clk);
            if (!empty) begin
                rd_en = 1;
            end
        end
        rd_en = 0;


        // 写入数据
        repeat(35) begin
            @(posedge wr_clk);
            if (!full) begin
                wr_en = 1;
                wr_data = wr_data + 1;
            end
        end
        wr_en = 0;

        // 延迟后开始读取
        #100;

        repeat(35) begin
            @(posedge rd_clk);
            if (!empty) begin
                rd_en = 1;
            end
        end
        rd_en = 0;


        #100 $finish;
    end

endmodule

module std_fifo (
    
    input wire rd_rst_n, // = clk 同一时钟域
    input wire rd_clk, // = clk 同一时钟域
    input wire rd_en,
    output wire empty,
    output reg [7:0] rd_data,

    input wire wr_clk,
    input wire wr_en,
    output wire full,
    input wire [7:0] wr_data

);
    
    (* ram_style = "block" *) reg [7:0] ram [15:0];

    // 写时钟域
    reg [4:0] wr_ptr = 0;
    always @(posedge wr_clk) begin
        if (!sync_rst_n_2) begin
            wr_ptr <= 0;
            sync_to_wr_1 <= 0;
            sync_to_wr_2 <= 0;
        end else if (wr_en && !full) begin
            wr_ptr <= wr_ptr + 1;
        end
    end
    always @(posedge wr_clk) if (wr_en && !full) ram[wr_ptr[3:0]] <= wr_data;
    wire [4:0] wr_gray = wr_ptr ^ (wr_ptr >> 1);
    reg [4:0] sync_to_wr_1; always @(posedge wr_clk) sync_to_wr_1 <= rd_gray;
    reg [4:0] sync_to_wr_2; always @(posedge wr_clk) sync_to_wr_2 <= sync_to_wr_1;
    assign full = (wr_gray == {~sync_to_wr_2[4], ~sync_to_wr_2[3], sync_to_wr_2[2:0]});
    
    reg sync_rst_n_1;
    reg sync_rst_n_2; 
    always @(posedge wr_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin    // 异步复位
            sync_rst_n_1 <= 0;
            sync_rst_n_2 <= 0;
        end else begin          // 同步释放
            sync_rst_n_1 <= 1;
            sync_rst_n_2 <= sync_rst_n_1;
        end
    end

    // 读时钟域
    reg [4:0] rd_ptr = 0;
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr <= 0;
            sync_to_rd_1 <= 0;
            sync_to_rd_2 <= 0;
        end else if (rd_en && !empty) begin
            rd_ptr <= rd_ptr + 1;
        end
    end
    always @(posedge rd_clk) if (rd_en && !empty) rd_data <= ram[rd_ptr[3:0]];
    wire [4:0] rd_gray = rd_ptr ^ (rd_ptr >> 1);
    reg [4:0] sync_to_rd_1; always @(posedge rd_clk) sync_to_rd_1 <= wr_gray;
    reg [4:0] sync_to_rd_2; always @(posedge rd_clk) sync_to_rd_2 <= sync_to_rd_1;
    assign empty = (rd_gray == sync_to_rd_2);


endmodule
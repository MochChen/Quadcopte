module bb_iic_tb;
    reg clk;
    reg rst_n;
    reg mpu_init;
    reg mpu_transfer;
    wire scl;
    wire sda;
    wire data_avalid;
    wire [7:0] data;
    wire busy_now;

    // Instantiate the module under test
    bb_iic uut (
        .clk(clk),
        .rst_n(rst_n),
        .mpu_init(mpu_init),
        .mpu_transfer(mpu_transfer),
        .scl(scl),
        .sda(sda),
        .data_avalid(data_avalid),
        .data(data),
        .busy_now(busy_now)
    );

    // Generate clock
    initial begin
        clk = 0;
        forever #10 clk = ~clk; // 50MHz clock
    end

    // Test sequence
    initial begin
        // Initialize signals
        rst_n = 0;
        mpu_init = 0;
        mpu_transfer = 0;

        // Reset sequence
        #50 rst_n = 1;
        
        // Test initialization
        #50 mpu_init = 1;
        #20 mpu_init = 0;

        // Wait for busy flag to clear
        wait (busy_now == 0);
        
        // Test data transfer
        #50 mpu_transfer = 1;
        #20 mpu_transfer = 0;

        // Wait for data to be valid
        wait (data_avalid == 1);

        // End simulation
        // test
        // meiyou genggai 
        #200;
        $stop;
    end
endmodule
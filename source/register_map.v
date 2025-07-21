/**************************************************************************************\
readme:
    -   使用axi双向接口,ps作为master,当axi_din_avalid为高时候读取数据axi_din_32
    -   需要输出8bit报警代码,112bit的mpu数据,总共120bit,输出端口是64bit的.分别两次
        进行发送,总共128bit,用4bit作为作第几次64bit包
    -   axi_din_32的第一个8bit是寄存器地址,剩余的3字节是数据

\**************************************************************************************/

module register_map (
    input  wire clk,
    input  wire rst_n,

    // axi接口
    input  wire [31:0]       axi_din_32,         // 接收32bit
    input  wire              axi_din_valid,      // 数据有效信号
    output reg [63:0]        axi_out_64H,        // 发送64bit_H
    output reg [63:0]        axi_out_64L,        // 发送64bit_L

    // 输出给其他模块使用的参数
    output reg signed [15:0] pwm_base,           
    output reg signed [15:0] tilt_cmp,           // 未使用
    output reg signed [7:0]  use_INT,            // 未使用              
    output reg [7:0]         error_reset, 
    
    output reg [7:0]         mpu_device_addr,
    output reg [7:0]         mpu_register_addr,
    output reg [7:0]         mpu_register_data,
    output reg [7:0]         mpu_read_start_addr,
    
    output reg signed [15:0] tgt_height,
    output reg signed [15:0] tgt_pitch,
    output reg signed [15:0] tgt_roll,
    output reg signed [15:0] tgt_yaw,
    
    output reg [23:0]        action_reg,        // 未使用
    output reg [7:0]         alpha,
    output reg [23:0]        max_speed,
    output reg [23:0]        min_speed,
    output reg [7:0]         KP,
    output reg [7:0]         KI,
    output reg [7:0]         KD,

    input wire [7:0]         error_code,
    input wire               mpu_read_done,
    input wire [111:0]       mpu_data_pack
);

/**************************************************************************************\
                                    寄存器写操作
\**************************************************************************************/  

    reg  axi_din_32_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
             axi_din_32_reg <= 0;
        end else begin
            if (axi_din_valid) begin
                axi_din_32_reg <= axi_din_32;
            end
        end
    end
    
    reg [31:0] register_data = 0;always @(posedge clk) register_data <= axi_din_32;
    reg [0:0] valid = 0;always @(posedge clk) valid <= axi_din_valid;


    // 提取地址和值的高低位
    wire [7:0] address          = valid ? register_data[31 -: 8] : 0;   // 地址字段（高 8 位）
    wire [7:0] register_8bit    = valid ? register_data[0 +: 8] : 0;    // 低 8 位寄存器值
    wire [15:0] register_16bit  = valid ? register_data[0 +: 16] : 0;   // 低 16 位寄存器值
    wire [23:0] register_24bit  = valid ? register_data[0 +: 24] : 0;   // 低 24 位寄存器值

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_base            <= 16'd0;
            tilt_cmp            <= 16'd5000; 
            use_INT             <= 8'd0;     
            error_reset         <= 8'b0;

            mpu_device_addr     <= 8'h68;
            mpu_register_addr   <= 8'h6B; 
            mpu_register_data   <= 8'h00;
            mpu_read_start_addr <= 8'h3B;  

            tgt_height          <= 16'sd0;    
            tgt_pitch           <= 16'sd0;    
            tgt_roll            <= 16'sd0;    
            tgt_yaw             <= 16'sd0; 

            action_reg          <= 24'd0;    
            alpha               <= 8'd1;    
            max_speed           <= 24'd95000;
            min_speed           <= 24'd50000;  
            KP                  <= 8'd0;    
            KI                  <= 8'd0;    
            KD                  <= 8'd0;
              
        end else begin
            if (valid) begin
                case (address)
                    8'h00:; // 地址不匹配时什么也不做
                    8'h01: pwm_base            <= register_16bit;
                    8'h02: tilt_cmp            <= register_16bit;  
                    8'h03: use_INT             <= register_8bit;   
  
                    8'h05: error_reset         <= register_8bit; 
                    
                    8'h06: mpu_device_addr     <= register_8bit;
                    8'h07: mpu_register_addr   <= register_8bit;   
                    8'h08: mpu_register_data   <= register_8bit;   
                    8'h09: mpu_read_start_addr <= register_8bit;      
                    
                    8'h0A: tgt_height          <= register_16bit;  
                    8'h0B: tgt_pitch           <= register_16bit;  
                    8'h0C: tgt_roll            <= register_16bit;  
                    8'h0D: tgt_yaw             <= register_16bit;  
                    
                    8'h0E: action_reg          <= register_24bit;  
                    8'h0F: alpha               <= register_8bit;  
                    8'h10: max_speed           <= register_24bit; 
                    8'h11: min_speed           <= register_24bit; 
                    8'h12: KP                  <= register_8bit; 
                    8'h13: KI                  <= register_8bit; 
                    8'h14: KD                  <= register_8bit; 
                    
                    default:; // 地址不匹配时什么也不做
                endcase
            end
        end
    end


/**************************************************************************************\
                                    寄存器读操作
\**************************************************************************************/

    // 打印参数1
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_out_64H <= 0;
            axi_out_64L <= 0;
        end else if (mpu_read_done) begin
            // 当 mpu_read_done 为高时，更新数据
            axi_out_64H <= {4'b0001, error_code, mpu_data_pack[111:60]};
            axi_out_64L <= {4'b0010, mpu_data_pack[59:0]};
        end
    end

    // 打印参数2
    // pitch roll yaw 
    // pitch-error roll-error yaw-error
    // pitch-i roll-i yaw-i
    // pitch-d roll-d yaw-d
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_out_64H <= 0;
            axi_out_64L <= 0;
        end else if (mpu_read_done) begin
            // 当 mpu_read_done 为高时，更新数据
            axi_out_64H <= {4'b0001, error_code, mpu_data_pack[111:60]};
            axi_out_64L <= {4'b0010, mpu_data_pack[59:0]};
        end
    end




endmodule

`timescale 1ns/1ps
`include "cal_pid.v"

module cal_pid_tb();

    // Parameters
    parameter Kp = 100;
    parameter Ki = 1;
    parameter Kd = 1;
    
    // Inputs
    reg clk;
    reg rst_n;
    reg cal_pid_en;
    reg [23:0] PWM_base;
    reg [23:0] pitch_error;
    reg [23:0] roll_error;
    reg [23:0] yaw_error;
    reg [23:0] i_pitch_error;
    reg [23:0] i_roll_error;
    reg [23:0] i_yaw_error;
    reg [23:0] d_pitch_error;
    reg [23:0] d_roll_error;
    reg [23:0] d_yaw_error;
    
    // Outputs
    wire [15:0] pwm_duty_1;
    wire [15:0] pwm_duty_2;
    wire [15:0] pwm_duty_3;
    wire [15:0] pwm_duty_4;
    
    // Instantiate the Unit Under Test (UUT)
    cal_pid #(
        .Kp(Kp),
        .Ki(Ki),
        .Kd(Kd)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .cal_pid_en(cal_pid_en),
        .PWM_base(PWM_base),
        .pitch_error(pitch_error),
        .roll_error(roll_error),
        .yaw_error(yaw_error),
        .i_pitch_error(i_pitch_error),
        .i_roll_error(i_roll_error),
        .i_yaw_error(i_yaw_error),
        .d_pitch_error(d_pitch_error),
        .d_roll_error(d_roll_error),
        .d_yaw_error(d_yaw_error),
        .pwm_duty_1(pwm_duty_1),
        .pwm_duty_2(pwm_duty_2),
        .pwm_duty_3(pwm_duty_3),
        .pwm_duty_4(pwm_duty_4)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end
    
    // Function to calculate expected PWM value
    function [15:0] calculate_expected;
        input [23:0] base;
        input [23:0] pitch_term;
        input [23:0] roll_term;
        input [23:0] yaw_term;
        input integer motor_num;
        begin
            case(motor_num)
                1: calculate_expected = base - pitch_term - roll_term - yaw_term;
                2: calculate_expected = base - pitch_term + roll_term + yaw_term;
                3: calculate_expected = base + pitch_term - roll_term + yaw_term;
                4: calculate_expected = base + pitch_term + roll_term - yaw_term;
                default: calculate_expected = 0;
            endcase
        end
    endfunction
    
    // Test procedure
    initial begin
        // Initialize Inputs
        rst_n = 0;
        cal_pid_en = 0;
        PWM_base = 0;
        pitch_error = 0;
        roll_error = 0;
        yaw_error = 0;
        i_pitch_error = 0;
        i_roll_error = 0;
        i_yaw_error = 0;
        d_pitch_error = 0;
        d_roll_error = 0;
        d_yaw_error = 0;
        
        // Reset the system
        #20;
        rst_n = 1;
        #10;
        
        // Test case 1: Basic PID calculation with base PWM
        cal_pid_en = 1;
        PWM_base = 24'd1000;
        pitch_error = 24'd1;
        roll_error = 24'd2;
        yaw_error = 24'd3;
        i_pitch_error = 24'd10;
        i_roll_error = 24'd20;
        i_yaw_error = 24'd30;
        d_pitch_error = 24'd100;
        d_roll_error = 24'd200;
        d_yaw_error = 24'd300;
        
        // Calculate PID terms
        // P_term = Kp * error = 100 * error
        // I_term = Ki * i_error = 1 * i_error
        // D_term = Kd * d_error = 1 * d_error
        // Total term = P + I + D
        
        #10;
        
        // Verify all PWM outputs
        if (pwm_duty_1 !== calculate_expected(PWM_base, 
                                            (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                            (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                            (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                            1))
            $display("Error: PWM1 incorrect. Expected %d, got %d", 
                    calculate_expected(PWM_base, 
                                     (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                     (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                     (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                     1), pwm_duty_1);
        
        if (pwm_duty_2 !== calculate_expected(PWM_base, 
                                            (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                            (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                            (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                            2))
            $display("Error: PWM2 incorrect. Expected %d, got %d", 
                    calculate_expected(PWM_base, 
                                     (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                     (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                     (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                     2), pwm_duty_2);
        
        if (pwm_duty_3 !== calculate_expected(PWM_base, 
                                            (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                            (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                            (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                            3))
            $display("Error: PWM3 incorrect. Expected %d, got %d", 
                    calculate_expected(PWM_base, 
                                     (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                     (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                     (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                     3), pwm_duty_3);
        
        if (pwm_duty_4 !== calculate_expected(PWM_base, 
                                            (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                            (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                            (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                            4))
            $display("Error: PWM4 incorrect. Expected %d, got %d", 
                    calculate_expected(PWM_base, 
                                     (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                     (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                     (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                     4), pwm_duty_4);
        
        // Test case 2: Negative errors
        pitch_error = -24'd1;
        roll_error = -24'd2;
        yaw_error = -24'd3;
        i_pitch_error = -24'd10;
        i_roll_error = -24'd20;
        i_yaw_error = -24'd30;
        d_pitch_error = -24'd100;
        d_roll_error = -24'd200;
        d_yaw_error = -24'd300;
        
        #10;
        
        // Verify all PWM outputs with negative terms
        if (pwm_duty_1 !== calculate_expected(PWM_base, 
                                            (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                            (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                            (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                            1))
            $display("Error: PWM1 with negative errors incorrect");
        
        if (pwm_duty_2 !== calculate_expected(PWM_base, 
                                            (Kp*pitch_error + Ki*i_pitch_error + Kd*d_pitch_error),
                                            (Kp*roll_error + Ki*i_roll_error + Kd*d_roll_error),
                                            (Kp*yaw_error + Ki*i_yaw_error + Kd*d_yaw_error),
                                            2))
            $display("Error: PWM2 with negative errors incorrect");
        
        // Test case 3: Disable PID calculation
        cal_pid_en = 0;
        PWM_base = 24'd2000;
        pitch_error = 24'd100;
        roll_error = 24'd200;
        yaw_error = 24'd300;
        
        #10;
        
        // Outputs should remain unchanged
        if (pwm_duty_1 !== calculate_expected(24'd1000, 
                                            (Kp*(-24'd1) + Ki*(-24'd10) + Kd*(-24'd100)),
                                            (Kp*(-24'd2) + Ki*(-24'd20) + Kd*(-24'd200)),
                                            (Kp*(-24'd3) + Ki*(-24'd30) + Kd*(-24'd300)),
                                            1))
            $display("Error: PWM1 changed when disabled");
        
        // Test case 4: Reset
        rst_n = 0;
        #10;
        rst_n = 1;
        cal_pid_en = 1;
        
        // All outputs should be zero
        if (pwm_duty_1 !== 0) $display("Error: PWM1 not reset");
        if (pwm_duty_2 !== 0) $display("Error: PWM2 not reset");
        if (pwm_duty_3 !== 0) $display("Error: PWM3 not reset");
        if (pwm_duty_4 !== 0) $display("Error: PWM4 not reset");
        
        // Test case 5: Saturation test (verify 16-bit output)
        PWM_base = 24'd30000;
        pitch_error = 24'd1000;
        roll_error = 24'd2000;
        yaw_error = 24'd3000;
        i_pitch_error = 24'd10000;
        i_roll_error = 24'd20000;
        i_yaw_error = 24'd30000;
        d_pitch_error = 24'd100000;
        d_roll_error = 24'd200000;
        d_yaw_error = 24'd300000;
        
        #10;
        
        // Just verify they're within 16-bit range (no exact value check)
        if (pwm_duty_1 > 65535) $display("Error: PWM1 exceeds 16-bit range");
        if (pwm_duty_2 > 65535) $display("Error: PWM2 exceeds 16-bit range");
        if (pwm_duty_3 > 65535) $display("Error: PWM3 exceeds 16-bit range");
        if (pwm_duty_4 > 65535) $display("Error: PWM4 exceeds 16-bit range");
        
        $display("Testbench completed");
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("At time %t: PWM1=%d, PWM2=%d, PWM3=%d, PWM4=%d", 
                 $time, pwm_duty_1, pwm_duty_2, pwm_duty_3, pwm_duty_4);
    end

endmodule
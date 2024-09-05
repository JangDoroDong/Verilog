`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/07/16 14:15:29
// Design Name: 
// Module Name: test_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module board_led_switch_test_top(
    input [15:0] switch,
    output [15:0] led);

    assign led = switch;
endmodule


module fnd_test_top(
    input clk, reset_p,
    input [15:0] switch,
    output [3:0] com,
    output [7:0] seg_7);
    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(switch), .com(com), .seg_7(seg_7));
endmodule


module watch_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7);
    
    // wire
    wire btn_mode;
    wire btn_sec;
    wire btn_min;
    wire set_watch;
    wire inc_sec, inc_min; // inc = increment
    wire clk_usec, clk_msec, clk_sec, clk_min;
    wire [3:0] sec1, sec10, min1, min10;
    wire [15:0] value;
    
    // button Mode    
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_mode));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_sec));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_min));
    
    // button control
    T_flip_flop_p t_mode(.clk(clk), .reset_p(reset_p), .t(btn_mode), .q(set_watch));
    assign inc_sec = set_watch ? btn_sec : clk_sec; 
    assign inc_min = set_watch ? btn_min : clk_min;
    
    // Time Setting
    clock_div_100 usec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    clock_div_1000 msec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_usec), .clk_div_1000(clk_msec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_msec), .clk_div_1000_nedge(clk_sec));
    clock_div_60 min_clk(.clk(clk), .reset_p(reset_p), .clk_source(inc_sec), .clk_div_60_nedge(clk_min));
      
    counter_bcd_60 counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(inc_sec), .bcd1(sec1), .bcd10(sec10));
    counter_bcd_60 counter_min(.clk(clk), .reset_p(reset_p), .clk_time(inc_min), .bcd1(min1), .bcd10(min10));
    
    assign value = {min10, min1, sec10, sec1};
    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
    
endmodule


module loadable_watch_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7);
    
    // wire
    wire btn_mode;
    wire btn_sec;
    wire btn_min;
    wire set_watch;
    wire inc_sec, inc_min; // inc = increment
    wire clk_usec, clk_msec, clk_sec, clk_min;
    
    // button Mode    
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_mode));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_sec));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_min));
    
    // button control
    T_flip_flop_p t_mode(.clk(clk), .reset_p(reset_p), .t(btn_mode), .q(set_watch));
    
    wire clk_source_nedge;
    edge_detector_n ed_source(.clk(clk), .reset_p(reset_p), .cp(set_watch), .p_edge(set_load_en), .n_edge(watch_load_en));
    
    assign inc_sec = set_watch ? btn_sec : clk_sec; 
    assign inc_min = set_watch ? btn_min : clk_min;
    
    // Time Setting
    clock_div_100 usec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    clock_div_1000 msec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_usec), .clk_div_1000(clk_msec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_msec), .clk_div_1000_nedge(clk_sec));
    clock_div_60 min_clk(.clk(clk), .reset_p(reset_p), .clk_source(inc_sec), .clk_div_60_nedge(clk_min));
      
    loadable_counter_bcd_60 sec_watch(
        .clk(clk), .reset_p(reset_p),
        .clk_time(clk_sec),
        .load_enable(watch_load_en),
        .load_bcd1(set_sec1), .load_bcd10(set_sec10),
        .bcd1(watch_sec1), .bcd10(watch_sec10));
         
    loadable_counter_bcd_60 min_watch(
        .clk(clk), .reset_p(reset_p),
        .clk_time(clk_min),
        .load_enable(watch_load_en),
        .load_bcd1(set_min1), .load_bcd10(set_min10),
        .bcd1(watch_min1), .bcd10(watch_min10)); 
        
    loadable_counter_bcd_60 sec_set(
        .clk(clk), .reset_p(reset_p),
        .clk_time(btn_sec),
        .load_enable(set_load_en),
        .load_bcd1(watch_sec1), .load_bcd10(watch_sec10),
        .bcd1(set_sec1), .bcd10(set_sec10));
         
    loadable_counter_bcd_60 min_set(
        .clk(clk), .reset_p(reset_p),
        .clk_time(btn_min),
        .load_enable(set_load_en),
        .load_bcd1(watch_min1), .load_bcd10(watch_min10),
        .bcd1(set_min1), .bcd10(set_min10));
              
    wire [15:0] value, watch_value, set_value;              
    wire [3:0] watch_sec1, watch_sec10, watch_min1, watch_min10;
    wire [3:0] set_sec1, set_sec10, set_min1, set_min10;    
    assign watch_value = {watch_min10, watch_min1, watch_sec10, watch_sec1};
    assign set_value = {set_min10, set_min1, set_sec10, set_sec1};
    assign value = set_watch ? set_value : watch_value;   
    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
    
endmodule


module stop_watch_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7,
    output led_start, led_lap);
    
    reg lap;
    wire start_stop;
    wire clk_start;
    wire btn_start, btn_lap, btn_clear;
    wire clk_usec, clk_msec, clk_sec, clk_min;
    wire reset_start;
    
    assign clk_start = start_stop ? clk : 0;
    
    // clock
    clock_div_100 usec_clk(.clk(clk_start), .reset_p(reset_start), .clk_div_100(clk_usec));
    clock_div_1000 msec_clk(.clk(clk_start), .reset_p(reset_start), .clk_source(clk_usec), .clk_div_1000(clk_msec));
    clock_div_1000 sec_clk(.clk(clk_start), .reset_p(reset_start), .clk_source(clk_msec), .clk_div_1000_nedge(clk_sec));
    clock_div_60 min_clk(.clk(clk_start), .reset_p(reset_start), .clk_source(clk_sec), .clk_div_60_nedge(clk_min));
    
    // button Mode    
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_lap));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_clear));
    
    assign reset_start = reset_p | btn_clear;
    
    // button/lap control
    T_flip_flop_p t_start(.clk(clk), .reset_p(reset_start), .t(btn_start), .q(start_stop));
    assign led_start = start_stop; // LED ON = START, LED OFF = STOP
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)lap = 0;
        else begin
            if(btn_lap) lap = ~lap;
            else if(btn_clear) lap = 0;
        end
    end    
    
    // Lap control
    assign led_lap = lap;
            
    wire [3:0] min10, min1, sec10, sec1;
    counter_bcd_60_clear counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(clk_sec), .clear(btn_clear), .bcd1(sec1), .bcd10(sec10));
    counter_bcd_60_clear counter_min(.clk(clk), .reset_p(reset_p), .clk_time(clk_min), .clear(btn_clear) , .bcd1(min1), .bcd10(min10));
    
    // Time Save Register
    reg [15:0] lap_time;
    wire [15:0] cur_time;
    assign cur_time = {min10, min1, sec10, sec1};
    always @(posedge clk or posedge reset_p)begin
        if(reset_p) lap_time = 0;
        else if(btn_lap) lap_time = cur_time;
        else if(btn_clear) lap_time = 0;
    end
        
    // Fnd controler
    wire [15:0] value;
    assign value = lap ? lap_time : cur_time;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
    
endmodule


module stop_watch_ms_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7,
    output led_start, led_lap);
    
    reg lap;
    wire start_stop;
    wire clk_start;
    wire btn_start, btn_lap, btn_clear;
    wire clk_usec, clk_msec, clk_10msec, clk_sec;
    wire reset_start;
    
    assign clk_start = start_stop ? clk : 0;
    
    // clock
    clock_div_100 usec_clk(.clk(clk_start), .reset_p(reset_start), .clk_div_100(clk_usec));
    clock_div_1000 msec_clk(.clk(clk_start), .reset_p(reset_start), .clk_source(clk_usec), .clk_div_1000(clk_msec));
    clock_div_10 cm_clk(.clk(clk_start), .reset_p(reset_start),.clk_source(clk_msec), .clk_div_10(clk_10msec));
    clock_div_1000 sec_clk(.clk(clk_start), .reset_p(reset_start), .clk_source(clk_msec), .clk_div_1000_nedge(clk_sec));
    
    // button Mode    
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_lap));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_clear));
    
    assign reset_start = reset_p | btn_clear;
    
    // button/lap control
    T_flip_flop_p t_start(.clk(clk), .reset_p(reset_start), .t(btn_start), .q(start_stop));
    assign led_start = start_stop; // LED ON = START, LED OFF = STOP
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)lap = 0;
        else begin
            if(btn_lap) lap = ~lap;
            else if(btn_clear) lap = 0;
        end
    end    
    
    // Lap control
    assign led_lap = lap;
            
    wire [3:0] sec10, sec1, mil10, mil1;
    counter_bcd_100_clear counter_mil(.clk(clk), .reset_p(reset_p), .clk_time(clk_10msec), .clear(btn_clear), .bcd1(mil1), .bcd10(mil10));
    counter_bcd_60_clear counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(clk_sec), .clear(btn_clear) , .bcd1(sec1), .bcd10(sec10));
    
    // Time Save Register
    reg [15:0] lap_time;
    wire [15:0] cur_time;
    assign cur_time = {sec10, sec1, mil10, mil1};
    always @(posedge clk or posedge reset_p)begin
        if(reset_p) lap_time = 0;
        else if(btn_lap) lap_time = cur_time;
        else if(btn_clear) lap_time = 0;
    end
        
    // Fnd controler
    wire [15:0] value;
    assign value = lap ? lap_time : cur_time;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
    
endmodule

module cook_timer_for_multi_top(
    input clk, reset_p,
    input [2:0] btn,
    input alarm_off,
    output [3:0] com,
    output [7:0] seg_7,
    output led_alarm, led_start, buzz
    );
    
    // Clock setting
    wire clk_usec, clk_msec, clk_sec, clk_min;
    clock_div_100 usec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    clock_div_1000 msec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_usec), .clk_div_1000(clk_msec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_msec), .clk_div_1000_nedge(clk_sec));
    clock_div_60 min_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_sec), .clk_div_60_nedge(clk_min));
    
    // Button setting    
    wire btn_start, btn_sec, btn_min, btn_alarmoff;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_sec));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_min));
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(alarm_off), .btn_pedge(btn_alarmoff));
    
    // 60 up counter
    wire [3:0] set_min10, set_min1, set_sec10, set_sec1;
    wire [3:0] cur_min10, cur_min1, cur_sec10, cur_sec1;
    counter_bcd_60 counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(btn_sec), .bcd1(set_sec1), .bcd10(set_sec10));
    counter_bcd_60 counter_min(.clk(clk), .reset_p(reset_p), .clk_time(btn_min), .bcd1(set_min1), .bcd10(set_min10));
    
    // 60 down counter
    wire dec_clk;
    loadable_down_counter_bcd_60 cur_sec(.clk(clk), .reset_p(reset_p), .clk_time(clk_sec), .load_enable(btn_start),
    .load_bcd1(set_sec1), .load_bcd10(set_sec10), .bcd1(cur_sec1), .bcd10(cur_sec10), .dec_clk(dec_clk));
    loadable_down_counter_bcd_60 cur_min(.clk(clk), .reset_p(reset_p), .clk_time(dec_clk), .load_enable(btn_start),
    .load_bcd1(set_min1), .load_bcd10(set_min10), .bcd1(cur_min1), .bcd10(cur_min10));
        
    // FND control
    wire [15:0] value, set_time, cur_time;
    assign set_time = {set_min10, set_min1, set_sec10, set_sec1};
    assign cur_time = {cur_min10, cur_min1, cur_sec10, cur_sec1};
    
    // Start/Set Toggle (T F/F)
    reg start_set, alarm;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            start_set = 0;
            alarm = 0;
        end
        else begin
            if(btn_start) start_set = ~start_set;
            else if(cur_time == 0 && start_set)begin
                start_set = 0;
                alarm = 1;
            end
            else if(btn_alarmoff) alarm = 0;
        end
    end
    
    // LED setting
    assign led_alarm = alarm;
    assign buzz = alarm;
    assign led_start = start_set;

    assign value = start_set ? cur_time : set_time;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7)); 
endmodule

module cook_timer_top(
    input clk, reset_p,
    input [3:0] btn,
    output [3:0] com,
    output [7:0] seg_7,
    output led_alarm, led_start, buzz);
    
    // Clock setting
    wire clk_usec, clk_msec, clk_sec, clk_min;
    clock_div_100 usec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    clock_div_1000 msec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_usec), .clk_div_1000(clk_msec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_msec), .clk_div_1000_nedge(clk_sec));
    clock_div_60 min_clk(.clk(clk), .reset_p(reset_p), .clk_source(clk_sec), .clk_div_60_nedge(clk_min));
    
    // Button setting    
    wire btn_start, btn_sec, btn_min, btn_alarm_off;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_sec));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_min));
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(btn[3]), .btn_pedge(btn_alarm_off));
    
    // 60 up counter
    wire [3:0] set_min10, set_min1, set_sec10, set_sec1;
    wire [3:0] cur_min10, cur_min1, cur_sec10, cur_sec1;
    counter_bcd_60 counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(btn_sec), .bcd1(set_sec1), .bcd10(set_sec10));
    counter_bcd_60 counter_min(.clk(clk), .reset_p(reset_p), .clk_time(btn_min), .bcd1(set_min1), .bcd10(set_min10));
    
    // 60 down counter
    wire dec_clk;
    loadable_down_counter_bcd_60 cur_sec(.clk(clk), .reset_p(reset_p), .clk_time(clk_sec), .load_enable(btn_start),
    .load_bcd1(set_sec1), .load_bcd10(set_sec10), .bcd1(cur_sec1), .bcd10(cur_sec10), .dec_clk(dec_clk));
    loadable_down_counter_bcd_60 cur_min(.clk(clk), .reset_p(reset_p), .clk_time(dec_clk), .load_enable(btn_start),
    .load_bcd1(set_min1), .load_bcd10(set_min10), .bcd1(cur_min1), .bcd10(cur_min10));
        
    // FND control
    wire [15:0] value, set_time, cur_time;
    assign set_time = {set_min10, set_min1, set_sec10, set_sec1};
    assign cur_time = {cur_min10, cur_min1, cur_sec10, cur_sec1};
    
    // Start/Set Toggle (T F/F)
    reg start_set, alarm;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            start_set = 0;
            alarm = 0;
        end
        else begin
            if(btn_start) start_set = ~start_set;
            else if(cur_time == 0 && start_set)begin
                start_set = 0;
                alarm = 1;
            end
            else if(btn_alarm_off) alarm = 0;
        end
    end
    
    // LED setting
    assign led_alarm = alarm;
    assign buzz = alarm;
    assign led_start = start_set;

    assign value = start_set ? cur_time : set_time;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7)); 
endmodule


module keypad_test_top(
    input clk, reset_p,
    input [3:0] row,
    output [3:0] col,
    output [3:0] com,
    output [7:0] seg_7,
    output led_key_valid);

    wire [3:0] key_value;
    wire key_valid;

    keypad_cntr_FSM keypad(.clk(clk), .reset_p(reset_p), .row(row), .col(col), .key_value(key_value), .key_valid(key_valid));
    
    assign led_key_valid = key_valid;
    
    wire key_valid_p;
    
    edge_detector_p ed(.clk(clk), .reset_p(reset_p), .cp(key_valid), .p_edge(key_valid_p));
    
    reg [15:0] key_count;
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)key_count = 0;
        else if(key_valid_p)begin
            if(key_value == 1)key_count = key_count + 1;
            else if(key_value == 2)key_count = key_count - 1;
            else if(key_value == 3)key_count = key_count + 2;
        end
    end
    
    // FND Controler
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(key_count), .com(com), .seg_7(seg_7));
endmodule


module dht11_test_top(
    input clk, reset_p,
    inout dht11_data,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);
    
    wire [7:0] humidity, temperature;
    dht11_cntr(.clk(clk), .reset_p(reset_p), .dht11_data(dht11_data), .humidity(humidity), .temperature(temperature), .led_debug(led_debug));
    
    wire [15:0] humidity_bcd, temperature_bcd;
    bin_to_dec bcd_humi(.bin({4'b0, humidity}), .bcd(humidity_bcd));
    bin_to_dec bcd_tmpr(.bin({4'b0, temperature}), .bcd(temperature_bcd));
    
    wire [15:0] value;
    assign value = {humidity_bcd[7:0], temperature_bcd[7:0]};
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));    
    
endmodule


module sonic_test_top(
    input clk, reset_p,
    input  echo,
    output trig,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);
    
    wire [21:0]  distance;
    HC_SR04_cntr (.clk(clk), .reset_p(reset_p), .echo(echo), .trig(trig), .distance(distance), .led_debug(led_debug));
    
    wire [15:0] trig_bcd;
    bin_to_dec bcd_humi(.bin(distance), .bcd(trig_bcd));
    
    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(trig_bcd), .com(com), .seg_7(seg_7));    
    
endmodule

module led_pwm_top(
    input clk, reset_p,
    output pwm, led_r, led_g, led_b);

    reg [31:0] clk_div;
    always @(posedge clk) clk_div = clk_div + 1;

    pwm_100step pwm_inst(.clk(clk), .reset_p(reset_p), .duty(clk_div[25:19]), .pwm(pwm));
     
     // RGB 제어
    pwm_Nstep_freq #(.duty_step(77)) pwm_r(.clk(clk), .reset_p(reset_p), .duty(clk_div[28:23]), .pwm(led_r));  
    pwm_Nstep_freq #(.duty_step(93)) pwm_g(.clk(clk), .reset_p(reset_p), .duty(clk_div[27:22]), .pwm(led_g));     
    pwm_Nstep_freq #(.duty_step(103)) pwm_b(.clk(clk), .reset_p(reset_p), .duty(clk_div[26:21]), .pwm(led_b));

endmodule


module dc_motor_pwm_top(
    input clk, reset_p,
    output motor_pwm,
    output [3:0] com,
    output [7:0] seg_7);
    
    reg [31:0] clk_div;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)clk_div = 0;
        else clk_div = clk_div + 1;
    end
    
    wire clk_div_26_nedge;
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(clk_div[26]), .n_edge(clk_div_26_nedge));
    
    reg [5:0] duty;
    always @(posedge clk or posedge reset_p) begin
        if(reset_p) duty = 20;
        else if(clk_div_26_nedge)begin
            if(duty >= 99) duty = 20;
            else duty = duty + 1; 
        end
    end
    
    pwm_Nstep_freq #(
        .duty_step(100),
        .pwm_freq(100))
    pwm_motor(
    .clk(clk), 
    .reset_p(reset_p), 
    .duty(duty),
    .pwm(motor_pwm));
    
    wire [15:0] duty_bcd;
    bin_to_dec bcd_humi(.bin({10'b0, clk_div[31:26]}), .bcd(duty_bcd));    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(duty_bcd), .com(com), .seg_7(seg_7));    
    
endmodule


module servo_motor_pwm_top(
    input clk, reset_p,
    input [3:0] btn,
    output motor_pwm,
    output [3:0] com,
    output [7:0] seg_7);
    
    reg [31:0] clk_div;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)clk_div = 0;
        else clk_div = clk_div + 1;
    end
    
    wire angle_0, angle_90, angle_180, angle;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(angle_0));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(angle_90));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(angle_180));
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(btn[3]), .btn_pedge(angle));
    
    wire clk_div_nedge;
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(clk_div[25]), .n_edge(clk_div_nedge));
    
    reg [5:0] duty;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) duty = 2;
        else if (angle_0) duty = 2;    // 0도 버튼 눌렸을 때
        else if (angle_90) duty = 6;   // 90도 버튼 눌렸을 때
        else if (angle_180) duty = 11; // 180도 버튼 눌렸을 때
        else if(angle) begin
            duty = duty + 2;
            if(duty >= 12) duty = 2;
        end
    end
    
    pwm_Nstep_freq #(
        .duty_step(100),
        .pwm_freq(50))
    pwm_motor(
    .clk(clk), 
    .reset_p(reset_p), 
    .duty(duty),
    .pwm(motor_pwm));
    
    wire [15:0] duty_bcd;
    bin_to_dec bcd_humi(.bin({6'b0, duty}), .bcd(duty_bcd));    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(duty_bcd), .com(com), .seg_7(seg_7));   

endmodule

////Homecurity전용 XADC
module adc_ch15_top(
    input clk, reset_p,
    input vauxp15, vauxn15,
    output reg buzz,        // Buzz 신호 출력
    output [3:0] com,
    output [7:0] seg_7
);
    
    wire [4:0] channel_out;
    wire [15:0] do_out;
    wire eoc_out;

    // XADC 인스턴스
    xadc_wiz_0 adc_15 (
        .daddr_in({2'b0, channel_out}),           // Address bus for the dynamic reconfiguration port
        .dclk_in(clk),                            // Clock input for the dynamic reconfiguration port
        .den_in(eoc_out),                         // Enable Signal for the dynamic reconfiguration port
        .reset_in(reset_p),                       // Reset signal for the System Monitor control logic
        .vauxp15(vauxp15),                        // Auxiliary channel 15 positive input
        .vauxn15(vauxn15),                        // Auxiliary channel 15 negative input (ground)
        .channel_out(channel_out),                // Channel Selection Outputs
        .do_out(do_out),                          // Output data bus for dynamic reconfiguration port
        .eoc_out(eoc_out)                         // End of Conversion Signal
    );

    // ADC 값을 판별하여 불꽃 감지 여부를 설정
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            buzz <= 0;
        end else begin
            // ADC 값이 100 이상이면 buzz를 1로 설정
            if (do_out[15:8] >= 100) begin
                buzz <= 1;
            end else begin
                buzz <= 0;
            end
        end
    end

    // Convert ADC binary value to BCD for FND display
    wire [15:0] adc_value;
    bin_to_dec bcd_adc (
        .bin({2'b0, do_out[15:6]}), // Convert ADC value to BCD
        .bcd(adc_value)
    );

    // FND display for ADC value
    fnd_cntr fnd (
        .clk(clk),
        .reset_p(reset_p),
        .value(adc_value),
        .com(com),
        .seg_7(seg_7)
    );

endmodule




module adc_ch6_top(
    input clk, reset_p,
    input vauxp6, vauxn6,
    output [3:0] com,
    output [7:0] seg_7,
    output led_pwm);
    
    wire [4:0] channel_out;
    wire [15:0] do_out;
    wire eoc_out;
    xadc_wiz_0 adc_6
              (
              .daddr_in({2'b0, channel_out}),            // Address bus for the dynamic reconfiguration port
              .dclk_in(clk),             // Clock input for the dynamic reconfiguration port
              .den_in(eoc_out),              // Enable Signal for the dynamic reconfiguration port
              .reset_in(reset_p),            // Reset signal for the System Monitor control logic
              .vauxp6(vauxp6),              // Auxiliary channel 6
              .vauxn6(vauxn6), //접지
              .channel_out(channel_out),         // Channel Selection Outputs
              .do_out(do_out),              // Output data bus for dynamic reconfiguration port
              .eoc_out(eoc_out)             // End of Conversion Signal
              );

    pwm_Nstep_freq #(
        .duty_step(256),
        .pwm_freq(10000)) // LED니까 Freq는 10000
    pwm_backlight(
    .clk(clk), 
    .reset_p(reset_p), 
    .duty(do_out[15:8]),
    .pwm(led_pwm));

    wire [15:0] adc_value;
    bin_to_dec bcd_adc(.bin({2'b0, do_out[15:6]}), .bcd(adc_value));    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(adc_value), .com(com), .seg_7(seg_7));              
              
endmodule


module adc_sequence2_top(
    input clk, reset_p,
    input vauxp6, vauxn6, vauxp15, vauxn15,
    output led_r, led_g,
    output [3:0] com,
    output [7:0] seg_7
);

    wire [4:0] channel_out;
    wire [15:0] do_out;
    wire eoc_out;
    xadc_wiz_1 adc_seq2
              (
              .daddr_in({2'b0, channel_out}),            // Address bus for the dynamic reconfiguration port
              .dclk_in(clk),             // Clock input for the dynamic reconfiguration port
              .den_in(eoc_out),              // Enable Signal for the dynamic reconfiguration port
              .reset_in(reset_p),            // Reset signal for the System Monitor control logic
              .vauxp6(vauxp6),              // Auxiliary channel 6
              .vauxn6(vauxn6), //접지
              .vauxp15(vauxp15),              // Auxiliary channel 15
              .vauxn15(vauxn15), //접지
              .channel_out(channel_out),         // Channel Selection Outputs
              .do_out(do_out),              // Output data bus for dynamic reconfiguration port
              .eoc_out(eoc_out)             // End of Conversion Signal
              );
              
    wire eoc_out_pedge;
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(eoc_out), .p_edge( eoc_out_pedge));
    
    reg [11:0] adc_value_x, adc_value_y;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p) begin
            adc_value_x = 0;
            adc_value_y = 0;
        end
        else if(eoc_out_pedge)begin
            case(channel_out[3:0])
                6 : adc_value_x = do_out[15:4];
                15 : adc_value_y = do_out[15:4];
            endcase
        end 
    end  
    
     pwm_Nstep_freq #(
        .duty_step(256),
        .pwm_freq(10000)) // LED니까 Freq는 10000
    pwm_red(
    .clk(clk), 
    .reset_p(reset_p), 
    .duty(adc_value_x[11:4]),
    .pwm(led_r));
    
    pwm_Nstep_freq #(
        .duty_step(256),
        .pwm_freq(10000)) // LED니까 Freq는 10000
    pwm_green(
    .clk(clk), 
    .reset_p(reset_p), 
    .duty(adc_value_y[11:4]),
    .pwm(led_g));
    
    wire [15:0] bcd_x, bcd_y, value;
    bin_to_dec bcd_adc_x(.bin({6'b0, adc_value_x[11:6]}), .bcd(bcd_x));
    bin_to_dec bcd_adc_y(.bin({6'b0, adc_value_y[11:6]}), .bcd(bcd_y));    
    
    assign value = {bcd_x[7:0], bcd_y[7:0]};
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
                

endmodule

/*


///////////////////서보모터 부분////////////////////

module sv_motor_inc2(
    input clk, reset_p,
    input sw_v17,  // 스위치 입력 추가
    input disable_sv,  // 서보모터 동작을 멈추는 신호
    output sv_pwm
);

    reg [31:0] clk_div;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p)
            clk_div <= 0;
        else
            clk_div <= clk_div + 1;
    end

    wire clk_div_21_nedge;   

    edge_detector_n ed(
        .clk(clk),
        .reset_p(reset_p),
        .cp(clk_div[21]),
        .n_edge(clk_div_21_nedge)
    );

    reg [6:0] duty;       // duty 레지스터의 크기를 7비트로 설정
    reg down_up;        // 방향 제어를 위한 플래그
    reg [6:0] duty_min, duty_max;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            duty <= 31;       // 초기화 1ms (5% 듀티 사이클)
            down_up <= 0;  // 초기 방향 설정 (0: 증가, 1: 감소)
            duty_min <= 12;
            duty_max <= 50;
        end
        else if (clk_div_21_nedge) begin // 20ms 주기
            if (disable_sv || !sw_v17) begin
                // disable_sv 또는 sw_v17이 활성화되면 현재 duty 값을 유지
                duty <= duty; 
            end else begin
                if (!down_up) begin
                    if (duty < duty_max)  // 2ms (10%)에 도달하지 않았다면 증가
                        duty <= duty + 1;
                    else
                        down_up <= 1;  // 2ms에 도달하면 방향을 감소로 변경
                end else begin
                    if (duty > duty_min)  // 1ms (5%)에 도달하지 않았다면 감소
                        duty <= duty - 1;
                    else
                        down_up <= 0;  // 1ms에 도달하면 방향을 증가로 변경
                end
            end
        end        
    end
    pwm_Nstep_freq2 #(
        .duty_step(400),  // 100단계로 나눔
        .pwm_freq(50)     // PWM 주파수 50Hz
    ) pwm_motor(
        .clk(clk),
        .reset_p(reset_p),
        .duty(duty),
        .pwm(sv_pwm)
    );
   
endmodule



module pwm_Nstep_freq2  /////////////////////////////////////////////////Nstep_freq2로 고쳤음. 나중에 2 뺴야함. 
        #(
        parameter sys_clk_freq = 100_000_000,
        parameter pwm_freq = 10_000,
        parameter duty_step = 100,
        parameter temp = sys_clk_freq / duty_step / pwm_freq,
        parameter temp_half = temp / 2)  //미리 계산되어서 값으로 적용된다.
(
        input clk, reset_p,
        input [31:0] duty,
        output pwm);
                       
        integer cnt_sysclk;       
        wire clk_freqXstep;
        always @(negedge clk or posedge reset_p)begin
                if(reset_p)cnt_sysclk = 0;
                else begin
                        if(cnt_sysclk >= temp-1) cnt_sysclk = 0;
                        else cnt_sysclk = cnt_sysclk + 1;
                 end
        end
        
        assign clk_freqXstep = (cnt_sysclk < temp_half) ? 1 : 0; 
        wire clk_freqXstep_nedge;
        
          edge_detector_n ed(
             .clk(clk), .reset_p(reset_p), .cp(clk_freqXstep),
             .n_edge(clk_freqXstep_nedge));

         integer cnt_duty;
        
        always @(negedge clk or posedge reset_p)begin
                if(reset_p)cnt_duty = 0;
                else if(clk_freqXstep_nedge) begin
                    if(cnt_duty >= (duty_step-1))cnt_duty = 0;   
                    else cnt_duty = cnt_duty + 1;
                 end
        end
        
        assign pwm = (cnt_duty < duty) ? 1 : 0;    //0 10초 25% 10초 ~     <=이면 0프로가 사라짐 
        //1초에 100hz가 10번 반복  
         
endmodule

////////////////////////////////////////////////////선풍기 부분///////////////////////////////////////////////////////////////////

module fan_pwm_top(
    input clk, 
    input reset_p,
    input btn,
    input ir_input,          // IR 센서 입력
    input dec_clk,           // 타이머 종료 신호

    output motor_pwm,
    output [7:0] value,      // FND에 표시할 값 출력
    output reg led_r, led_g, led_b);

    wire btn_start;
    wire no_object_detected;      // 물체 감지가 되지 않음을 나타내는 신호
    reg reset_counter;            // IR 센서 타이머 리셋 신호
    reg [5:0] duty;               // 팬의 듀티 사이클 (모터 속도)
    reg [3:0] fnd_value;          // FND에 표시할 모터 속도 값

    // 버튼 처리
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn), .btn_pedge(btn_start));

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            duty <= 0;  // 리셋 시 모터 정지
            reset_counter <= 1'b1;  // 타이머 초기화
        end else if (dec_clk) begin
            duty <= 0;  // 타이머 종료 시 모터 정지
        end else if (btn_start) begin
            if (duty == 0) begin
                duty <= 21;  // 버튼을 누르면 기본 속도로 시작
                reset_counter <= 1'b1;  // 타이머 리셋
            end else if (duty >= 63) begin
                duty <= 0;   // 다시 버튼을 누르면 팬을 끔
                reset_counter <= 1'b0;  // 팬이 꺼질 때는 타이머 리셋 비활성화
            end else begin
                duty <= duty + 21;  // 팬 속도 증가
                reset_counter <= 1'b1;  // 타이머 리셋
            end
        end else if (duty != 0 && no_object_detected) begin
            // 팬이 동작 중이고, 물체가 5초 동안 감지되지 않으면 팬을 끔
            duty <= 0;  // 팬을 멈추기 위해 duty를 0으로 설정
            reset_counter <= 1'b0;  // 팬이 꺼질 때는 타이머 리셋 비활성화
        end else begin
            reset_counter <= 1'b0;  // 기본 상태에서 타이머 리셋 비활성화
        end
    end  

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            // 리셋 시 RGB LED를 초기화
            fnd_value <= 4'b0;
            led_r <= 0;
            led_g <= 0;
            led_b <= 0;
        end else begin
            if (duty == 0) begin
                fnd_value <= 4'b0;
                led_r <= 0;
                led_g <= 0;
                led_b <= 0;
            end else if (duty == 21) begin
                fnd_value <= 4'd1;
                led_r <= 1;
                led_g <= 0;
                led_b <= 0;
            end else if (duty == 42) begin
                fnd_value <= 4'd2;
                led_r <= 0;
                led_g <= 1;
                led_b <= 0;
            end else if (duty == 63) begin
                fnd_value <= 4'd3;
                led_r <= 0;
                led_g <= 0;
                led_b <= 1;            
            end
        end
    end

    // IR 센서 모듈 인스턴스화
    ir_sensor ir_sensor_inst (
        .clk(clk),
        .reset_p(reset_p),  // 시스템 리셋 신호
        .ir_input(ir_input),
        .count_enable(duty != 0),  // 팬이 켜졌을 때만 카운트 동작
        .reset_counter(reset_counter), // 버튼을 누를 때마다 타이머 리셋
        .no_object_detected(no_object_detected)  // 감지되지 않음을 나타내는 신호 연결
    );

    // 팬 제어를 위해 PWM 모듈 사용
    pwm_Nstep_freq2 #(
        .duty_step(100),
        .pwm_freq(100)
    ) pwm_motor(
        .clk(clk),
        .reset_p(reset_p),
        .duty(duty),  // duty 값을 그대로 사용하여 PWM 신호 생성
        .pwm(motor_pwm)
    );

    // FND에 표시할 값을 BCD로 변환
    assign value = {4'b0000, fnd_value};  // 8비트로 확장하여 출력

endmodule




module ir_sensor (
    input clk, 
    input reset_p,
    input ir_input,         // FC-51 센서의 디지털 출력 입력
    input count_enable,     // 카운트 동작 제어 신호 (팬이 켜진 상태에서만 활성화)
    input reset_counter,    // 타이머 리셋 신호 (팬의 상태 변화 시 사용)
    output reg no_object_detected // 물체 감지 여부 출력
);
    // 5초 타이머를 위한 카운터 (100 MHz 클럭 가정, 5초 = 500,000,000 사이클)
    reg [28:0] no_object_counter; // 29비트 카운터 (2^29 > 500,000,000)
    localparam TIMEOUT = 29'd500_000_000; // 5초 (100MHz 클럭 기준)

    always @(posedge clk or posedge reset_p or posedge reset_counter) begin
        if (reset_p || reset_counter) begin
            no_object_counter <= 29'd0;
            no_object_detected <= 1'b0;
        end else if (!count_enable) begin
            // 카운트가 비활성화 상태라면 카운터와 감지 상태 초기화
            no_object_counter <= 29'd0;
            no_object_detected <= 1'b0;
        end else begin
            if (ir_input == 1'b0) begin
                // 물체가 감지되면 카운터 초기화
                no_object_counter <= 29'd0;
                no_object_detected <= 1'b0;
            end else if (no_object_counter < TIMEOUT) begin
                // 물체가 감지되지 않으면 카운터 증가
                no_object_counter <= no_object_counter + 1;
            end else begin
                // 5초 동안 물체가 감지되지 않으면 감지 신호 출력
                no_object_detected <= 1'b1;
            end
        end
    end
endmodule


module fan_timer(
    input clk, reset_p,
    input btn,  // btn[2] 사용  
    output [7:0] value,
    output dec_clk         // 타이머가 0에 도달했음을 알리는 신호
);

    wire clk_microsec, clk_millisec, clk_sec;
    reg [3:0] set_sec1, set_sec10;
    wire [3:0] cur_sec1, cur_sec10;  // 현재 초 값을 저장하는 레지스터
    reg [1:0] state;

    // Clocks
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_microsec));
    clock_div_1000 millisec_clk(.clk(clk), .reset_p(reset_p), 
        .clk_source(clk_microsec), .clk_div_1000(clk_millisec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p), 
        .clk_source(clk_millisec), .clk_div_1000_nedge(clk_sec)); 
    
    // Edge detector for button press
    wire btn_nedge;
    button_cntr btn_debounce(.clk(clk), .reset_p(reset_p), .btn(btn), .btn_nedge(btn_nedge));

    // Loadable down counters
    fan_down_counter cur_sec(
        .clk(clk), 
        .reset_p(reset_p), 
        .clk_time(clk_sec), 
        .load_enable(btn_nedge), 
        .load_bcd1(set_sec1), 
        .load_bcd10(set_sec10), 
        .bcd1(cur_sec1),     // 현재 1초 단위 (1의 자리)
        .bcd10(cur_sec10),   // 현재 10초 단위 (10의 자리)
        .dec_clk(dec_clk)    // 타이머 종료 신호
    );

    // Timer setting states
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state <= 2'b00;
            set_sec1 <= 4'b0011; // 초기값 3초
            set_sec10 <= 4'b0000;
        end else begin
            if (btn_nedge) begin
                // 버튼을 누르면 상태를 변경
                case (state)
                    2'b00: begin
                        set_sec1 <= 4'b0101; // 다음 상태: 5초
                        set_sec10 <= 4'b0000;
                        state <= 2'b01;
                    end
                    2'b01: begin
                        set_sec1 <= 4'b0000; // 다음 상태: 10초
                        set_sec10 <= 4'b0001;
                        state <= 2'b10;
                    end
                    2'b10: begin
                        set_sec1 <= 4'b0011; // 다음 상태: 3초
                        set_sec10 <= 4'b0000;
                        state <= 2'b00;
                    end
                    default: begin
                        state <= 2'b00;
                    end
                endcase
            end else if (dec_clk) begin
                // 타이머가 0에 도달하면 초기 상태로 되돌림
                set_sec1 <= 4'b0011; // 3초로 초기화
                set_sec10 <= 4'b0000;
                state <= 2'b00;
            end
        end
    end

    // 타이머 값을 상위 비트에 할당하여 표시
    assign value = {cur_sec10, cur_sec1};
endmodule



module fan_down_counter(
    input clk, 
    input reset_p,
    input clk_time,
    input load_enable,
    input [3:0] load_bcd1, 
    input [3:0] load_bcd10,
    output reg [3:0] bcd1, 
    output reg [3:0] bcd10,
    output reg dec_clk  // 타이머 종료 신호 출력
);

    wire clk_time_nedge;
    edge_detector_n ed_clk(
        .clk(clk), 
        .reset_p(reset_p), 
        .cp(clk_time),
        .n_edge(clk_time_nedge)
    );

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            bcd1 <= 0;
            bcd10 <= 0;
            dec_clk <= 0;
        end else if (load_enable) begin
            bcd1 <= load_bcd1;
            bcd10 <= load_bcd10;
            dec_clk <= 0; // Load 시에는 dec_clk를 리셋
        end else if (clk_time_nedge) begin
            if (bcd1 == 0 && bcd10 == 0) begin
                dec_clk <= 1; // 카운터가 0에 도달하면 dec_clk를 1로 설정하여 종료 신호로 사용
            end else begin
                if (bcd1 == 0) begin                
                    bcd1 <= 9;
                    if (bcd10 > 0) begin
                        bcd10 <= bcd10 - 1;
                    end
                end else begin
                    bcd1 <= bcd1 - 1;
                end
                dec_clk <= 0; // 아직 카운트 중이므로 dec_clk를 0으로 유지
            end
        end
    end
endmodule

//////////LED///////////

module led_abc_top (
    input clk, 
    input reset_p,
    input btn,               // 단일 입력으로 btn 사용
    output [15:0] led_debug
);

    reg [1:0] brightness_level; // 2비트 밝기 레벨
    reg [6:0] duty_cycle;
    wire pwm_signal;
    wire btn_led;              // 디바운싱된 버튼 신호 (펄스 에지)
    reg btn_last_state;

    // 버튼 컨트롤러 모듈 인스턴스화 (btn을 처리)
    button_cntr btn0 (
        .clk(clk), 
        .reset_p(reset_p), 
        .btn(btn), 
        .btn_pedge(btn_led)   // 디바운싱된 버튼 신호 출력 (이름 변경)
    );

    // 밝기 단계에 따른 duty cycle 설정
    always @(*) begin
        case (brightness_level)
            2'b00: duty_cycle = 7'd0;   // LED 꺼짐
            2'b01: duty_cycle = 7'd25;  // LED 약한 밝기
            2'b10: duty_cycle = 7'd50;  // LED 중간 밝기
            2'b11: duty_cycle = 7'd75;  // LED 최대 밝기
            default: duty_cycle = 7'd0;
        endcase
    end

    // 밝기 레벨 변경 - 디바운싱된 버튼 신호 (펄스 에지 감지)
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            brightness_level <= 2'b00;  // 초기 밝기 레벨
        end else if (btn_led) begin
            brightness_level <= brightness_level + 1;  // 밝기 단계 증가
        end
    end

    // PWM 모듈 인스턴스
    pwm_100step pwm_inst(
        .clk(clk), 
        .reset_p(reset_p), 
        .duty(duty_cycle), 
        .pwm(pwm_signal)
    );

    // 모든 LED에 동일한 PWM 신호를 적용
    assign led_debug = {16{pwm_signal}};

endmodule


////////////////////////////////////////종합 선풍기 코드 구현 ///////////////////////////////////////////////////////////////////////////

module multi_fan(
    input clk, 
    input reset_p,
    input [3:0] btn,
    input ir_input,          // IR 센서 입력
    input sw_v17,            // 서보모터 제어 스위치 입력

    output motor_pwm,
    output sv_pwm,
    output [3:0] com,
    output [7:0] seg_7,
    output led_r, led_g, led_b,
    output [15:0] led_debug // 추가된 LED 디버깅 출력
);

    wire [3:0] cur_sec1, cur_sec10; // 타이머의 현재 초 (1의 자리, 10의 자리)
    reg dec_clk, dec_clk_latched;   // 타이머 종료 신호 및 latched 신호
    wire motor_is_stopped;

    // 서보모터 제어 모듈 인스턴스화
    sv_motor_inc2 sv_motor_inst (
        .clk(clk),
        .reset_p(reset_p),
        .sw_v17(sw_v17),
        .disable_sv(motor_is_stopped || dec_clk_latched),  // 추가된 신호 연결
        .sv_pwm(sv_pwm)
    );

    wire [7:0] motor_value;
    assign motor_is_stopped = (motor_value == 0);  // 모터 듀티가 0인지 확인

    // 팬 및 LED 제어 모듈 인스턴스화
    fan_pwm_top fan_pwm_inst (
        .clk(clk), 
        .reset_p(reset_p),
        .btn(btn[0]),
        .ir_input(ir_input),
        .dec_clk(dec_clk_latched),  // 타이머 종료 신호 입력
        .motor_pwm(motor_pwm),
        .value(motor_value),        // 모터의 현재 값 출력
        .led_r(led_r),
        .led_g(led_g),
        .led_b(led_b)
    );
    
    wire [7:0] timer_value;
    wire dec_clk1;  // 타이머 종료 신호
    // 타이머 모듈 인스턴스화
    fan_timer timer_inst (
        .clk(clk), 
        .reset_p(reset_p),
        .btn(btn[2]),               // 타이머 설정 버튼 (btn[2] 사용)
        .dec_clk(dec_clk1),         // 타이머 종료 신호
        .value(timer_value)
    );

    // dec_clk 신호를 한번 처리하고 latched 신호로 변환
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            dec_clk <= 1'b0;
            dec_clk_latched <= 1'b0;
        end else begin
            if (dec_clk1 && !dec_clk) begin
                dec_clk_latched <= 1'b1;  // 타이머 종료 시 한번만 신호 발생
            end else if (!btn[0] && dec_clk_latched) begin
                dec_clk_latched <= 1'b0;  // 모터 버튼이 눌리면 dec_clk_latched 해제
            end
            dec_clk <= dec_clk1;
        end
    end

    // 최종 FND 값 결합 (타이머 값 상위, 모터 속도 하위)
    wire [15:0] final_fnd_value;
    assign final_fnd_value = {timer_value, motor_value};
    
    // FND 컨트롤러에 결합된 값 출력
    fnd_cntr fnd_combined(
        .clk(clk), 
        .reset_p(reset_p), 
        .value(final_fnd_value), 
        .com(com), 
        .seg_7(seg_7)
    );

    // LED 제어 모듈 인스턴스화
    led_abc_top led_abc_inst (
        .clk(clk),
        .reset_p(reset_p),
        .btn(btn[1]),         // btn[1]을 사용하여 LED 밝기 조절
        .led_debug(led_debug) // 16비트 LED 디버그 출력
    );

endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

module i2c_master_top(
    input clk, reset_p,
    input [1:0] btn,
    output scl, sda,
    output [15:0] led_debug
);

    reg [7:0] data;
    reg comm_go;

    wire [1:0] btn_pedge;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_pedge[0]));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_pedge[1]));
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            data = 0;
            comm_go = 0;
        end
        else begin
            if(btn_pedge[0])begin
                data = 8'b0000_0000;
                comm_go = 1;
            end
            else if(btn_pedge[1])begin
                data = 8'b0000_0100;
                comm_go = 1;
            end
            else comm_go = 0;
        end
    end
    
    I2C_master(.clk(clk), .reset_p(reset_p), .addr(7'h27), .rd_wr(0), .data(data), .comm_go(comm_go), .scl(scl), .sda(sda), .led_debug(led_debug));

endmodule


module i2c_txtlcd_top(
    input clk, reset_p,
    input [3:0] btn,
    output scl, sda,
    output [15:0] led_debug);

    parameter IDLE = 6'b00_0001;
    parameter INIT = 6'b00_0010;
    parameter SEND_DATA = 6'b00_0100;
    parameter SEND_COMMAND = 6'b00_1000;
    parameter SEND_STRING = 6'b01_0000;
    
     wire clk_usec;
    clock_div_100 usec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));
    
    reg [21:0] count_usec;
    reg count_usec_e;
    always @(negedge clk or posedge reset_p)begin
        if(reset_p) count_usec = 0;
        else if(clk_usec && count_usec_e) count_usec = count_usec + 1;
        else if(!count_usec_e) count_usec = 0;
    end
    
    wire [3:0] btn_pedge;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_pedge[0]));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_pedge[1]));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_pedge[2]));
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_pedge[3]));
    
    reg [7:0] send_buffer;
    reg rs, send;
    wire busy;
    i2c_lcd_send_byte txtlcd( .clk(clk), .reset_p(reset_p), .addr(7'h27), .send_buffer(send_buffer), .rs(rs), .send(send), .scl(scl), .sda(sda), .busy(busy), .led_debug(led_debug));
    
    reg [5:0] state, next_state;
    always @(negedge clk or posedge reset_p)begin
        if(reset_p) state = IDLE;
        else state = next_state;
    end
    
    reg init_flag;
    reg [3:0] cnt_data;
    reg [8*5-1:0] hello; // 다섯글자니까 8(bit) * 5
    reg [3:0] cnt_string;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            next_state = IDLE;
            init_flag = 0;
            cnt_data = 0;
            rs = 0;
            hello = "HELLO";
            cnt_string = 0;
        end
        else begin
            case(state)
                IDLE:begin
                    if(init_flag)begin
                        if(btn_pedge[0]) next_state = SEND_DATA;
                        if(btn_pedge[1]) next_state = SEND_COMMAND;
                        if(btn_pedge[2]) next_state = SEND_STRING;
                    end
                    else begin
                        if(count_usec <= 22'd80_000)begin
                            count_usec_e = 1;
                        end
                        else begin
                            next_state = INIT;
                            count_usec_e = 0;
                        end
                    end
                end
                INIT:begin
                    if(busy)begin
                        send = 0;
                        if(cnt_data >= 6)begin
                            next_state = IDLE;
                            init_flag = 1;
                            cnt_data = 0;
                        end
                    end
                    else if(!send)begin
                        case(cnt_data)
                            0 : send_buffer = 8'h33;
                            1 : send_buffer = 8'h32;
                            2 : send_buffer = 8'h28;
                            3 : send_buffer = 8'h0f;
                            4 : send_buffer = 8'h01;
                            5 : send_buffer = 8'h06;
                            endcase
                            rs = 0;
                            send = 1;
                            cnt_data = cnt_data + 1;
                    end
                end
                SEND_DATA:begin
                    if(busy)begin
                        next_state = IDLE;
                        send = 0;
                        if(cnt_data >= 9)cnt_data = 0;
                        else cnt_data = cnt_data + 1;
                    end
                    else begin
                        send_buffer = "0" + cnt_data;
                        rs = 1;
                        send = 1;
                    end
                end
                SEND_COMMAND:begin
                     if(busy)begin
                        next_state = IDLE;
                        send = 0;
                    end
                    else begin
                        send_buffer = 8'h18;
                        rs = 0;
                        send = 1;
                    end
                end
                SEND_STRING:begin
                     if(busy)begin
                        send = 0;
                        if(cnt_string >= 5)begin
                            next_state = IDLE;
                            cnt_string = 0;
                        end
                    end
                    else if(!send)begin
                        case(cnt_string)
                            0 : send_buffer = hello[39:32];
                            1 : send_buffer = hello[31:24];
                            2 : send_buffer = hello[23:16];
                            3 : send_buffer = hello[15:8];
                            4 : send_buffer = hello[7:0];
                            endcase
                            rs = 1;
                            send = 1;
                            cnt_string = cnt_string + 1;
                    end
                end
            endcase
        end
    end
    
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////PIR Sensor/////////

module pir_sensor(
    input clk, reset_p,
    input  pir_input,
    output reg pir_sensor);

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            pir_sensor = 0;    // 리셋 시 출력 초기화
        end else begin
            pir_sensor = pir_input;  // PIR 센서의 상태를 pir_sensor로 출력
        end
    end
endmodule

module pir_sensor_test(
    input clk, reset_p,
    input pir_input,
    output [3:0] com,
    output [7:0] seg_7
);

    wire pir_sensor;
    pir_sensor pir_sensor_inst(
        .clk(clk),
        .reset_p(reset_p),
        .pir_input(pir_input),
        .pir_sensor(pir_sensor)
    );
   
    wire [15:0] pir_value = {15'b0, pir_sensor};
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(pir_value), .com(com), .seg_7(seg_7));   

endmodule

//////////////자동 불켜짐(적외선 + 조도 + LED)//////////////

module auto_light(
    input clk, reset_p,
    input auto_switch, led_switch, // 스위치 변수 
    input pir_input,
    input vauxp6, vauxn6,
    output reg led_pwm_reg,  // auto 모드 LED 밝기 제어
    output reg led_onoff,    // 수동 모드  LED 제어
    output [3:0] com,
    output [7:0] seg_7);

    wire pir_sensor;
    wire led_pwm;
    reg [31:0] pir_timeout_counter; // 타이머 카운터
    reg pir_detected; // PIR 센서 감지 상태 플래그
    reg auto_mode; // auto_switch 활성화 상태를 유지

    // PIR 센서 모듈 인스턴스화
    pir_sensor pir_inst (
        .clk(clk),
        .reset_p(reset_p),
        .pir_input(pir_input),
        .pir_sensor(pir_sensor));

    // 조도 센서 모듈 인스턴스화
    adc_ch6_top adc_inst (
        .clk(clk),
        .reset_p(reset_p),
        .vauxp6(vauxp6),
        .vauxn6(vauxn6),
        .com(),
        .seg_7(),
        .led_pwm(led_pwm));

    // LED 제어
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            led_pwm_reg = 0;
            led_onoff = 0;
            pir_timeout_counter = 0;
            pir_detected = 0;
            auto_mode = 0;
        end 
        else begin
            // auto_switch가 켜져 있을 때
            if (auto_switch) begin 
                auto_mode = 1;    
                if (pir_sensor) begin
                    // PIR 센서가 감지되면 LED 밝기 조절
                    led_pwm_reg = led_pwm;
                    pir_detected = 1; // PIR 센서 감지됨
                    pir_timeout_counter = 0; // 타이머 리셋
                end 
                else begin
                    if (pir_detected) begin
                        // PIR 센서가 감지되지 않으면서 이전에 감지된 상태였던 경우
                        if (pir_timeout_counter >= 32'd833_333_333) begin // 10초동안 켜짐
                            // 10초 지났다면 LED 끄기
                            led_pwm_reg = 0;
                            auto_mode = 0;
                            pir_detected = 0; // 감지 상태 플래그 리셋
                        end 
                        else begin
                            pir_timeout_counter = pir_timeout_counter + 1; // 타이머 증가
                            led_pwm_reg = led_pwm; // PIR 센서가 감지되지 않으면서 이전에 감지된 상태일 때 LED 켜짐
                        end
                    end 
                    else begin
                        // PIR 센서가 감지되지 않았고 이전에 감지 상태도 아니었던 경우
                        led_pwm_reg = 0;
                    end
                end
            end 
            else begin
                // auto_switch가 꺼져 있을 때
                auto_mode = 0;
                if (led_switch) begin
                    // led_switch가 켜져 있을 때 LED 켜기
                    led_onoff = 1;
                end 
                else begin
                    // led_switch가 꺼져 있을 때 LED 끄기
                    led_onoff = 0;
                end
                led_pwm_reg = 0; // auto_mode가 꺼져 있으면 LED 제어를 auto 모드로 하지 않으므로 끔
            end
        end
    end
endmodule


///////////////////////////////현관등(적외선 센서 + LED)/////////////////////////////////////

module door_light(
    input clk, reset_p,
    input pir_input,
    output reg door_led);  // auto 모드 LED 밝기 제어);

    wire pir_sensor;
    reg [31:0] pir_timeout_counter; // 타이머 카운터
    reg pir_detected; // PIR 센서 감지 상태 플래그
    reg auto_mode; // auto_switch 활성화 상태를 유지

    // PIR 센서 모듈 인스턴스화
    pir_sensor pir_inst (
        .clk(clk),
        .reset_p(reset_p),
        .pir_input(pir_input),
        .pir_sensor(pir_sensor));

    // LED 제어
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            door_led = 0;
            pir_timeout_counter = 0;
            pir_detected = 0;
            auto_mode = 0;
        end 
        else begin
                if (pir_sensor) begin
                    // PIR 센서가 감지되면 LED 밝기 조절
                    door_led = 1;
                    pir_detected = 1; // PIR 센서 감지됨
                    pir_timeout_counter = 0; // 타이머 리셋
                end 
                else begin
                    if (pir_detected) begin
                        // PIR 센서가 감지되지 않으면서 이전에 감지된 상태였던 경우
                        if (pir_timeout_counter >= 32'd833_333_333) begin // 10초동안 켜짐
                            // 10초 지났다면 LED 끄기
                            door_led = 0;
                            auto_mode = 0;
                            pir_detected = 0; // 감지 상태 플래그 리셋
                        end 
                        else begin
                            pir_timeout_counter = pir_timeout_counter + 1; // 타이머 증가
                            door_led = 1; // PIR 센서가 감지되지 않으면서 이전에 감지된 상태일 때 LED 켜짐
                        end
                    end 
                    else begin
                        // PIR 센서가 감지되지 않았고 이전에 감지 상태도 아니었던 경우
                        door_led = 0;
                    end
                end
            end 
        end
endmodule


///////////////////////////////////거실불(조도 + LED) / 수동 LED////////////////////////////////////////////

module livingroom_light(
    input clk, reset_p,
    input auto_switch, led_switch, // 스위치 변수 
    input vauxp6, vauxn6,
    output reg livingroom_led,  // auto 모드 LED 밝기 제어
    output reg passive_led    // 수동 모드  LED 제어
    );

    wire led_pwm;
    reg auto_mode; // auto_switch 활성화 상태를 유지

    // 조도 센서 모듈 인스턴스화
    adc_ch6_top adc_inst (
        .clk(clk),
        .reset_p(reset_p),
        .vauxp6(vauxp6),
        .vauxn6(vauxn6),
        .com(),
        .seg_7(),
        .led_pwm(led_pwm));

    // LED 제어
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            livingroom_led = 0;
            passive_led = 0;
            auto_mode = 0;
        end 
        else begin
            // auto_switch가 켜져 있을 때
            if (auto_switch) livingroom_led = ~led_pwm; // PIR 센서가 감지되지 않으면서 이전에 감지된 상태일 때 LED 켜짐
            else if(!auto_switch)begin 
                livingroom_led = 0;
                if(led_switch) passive_led = 1;
                else if(!led_switch) passive_led = 0;
            end
        end
    end
endmodule


/////////////////////////////////방범모드 1/////////////////////////////////

module security_top(
    input clk, reset_p,
    input pir_input,
    output reg buzz
);
    wire pir_sensor;
    pir_sensor(.clk(clk), .reset_p(reset_p), .pir_input(pir_input), .pir_sensor(pir_sensor));
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            buzz = 0;
        end
        else if(pir_sensor)begin
            buzz = 1;
        end
        else buzz = 0;
    end
endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/07/25 17:02:53
// Design Name: 
// Module Name: multi_watch
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


module multi_watch(
    input clk, reset_p,
    input [3:0] btn,
    input alarm_off,
    output reg [3:0] com,
    output reg [7:0] seg_7,
    output reg led_start, led_lap, led_alarm, buzz
    );
    
    // 타이머 모듈 인스턴스
    wire [3:0] com_watch, com_stopwatch, com_timer;
    wire [7:0] seg_7_watch, seg_7_stopwatch, seg_7_timer;
    wire led_start_timer, led_start_stopwatch, led_lap_stopwatch, led_alarm_timer, alarm_off_timer;
    wire buzz_timer;
    reg [3:0] watch_btn, stop_watch_btn, cooktimer_btn;
   
   wire btn_off;
   button_cntr btn_m(.clk(clk), .reset_p(reset_p), .btn(btn[3]), .btn_pedge(btn_mode)); //버튼 모드 컨트롤러
   
    //  TIMER
    cook_timer_for_multi_top timer_instance (
        .clk(clk),
        .reset_p(reset_p),
        .btn(cooktimer_btn),
        .alarm_off(alarm_off), // 외부 버튼으로 타이머 알람 제어
        .com(com_timer),
        .seg_7(seg_7_timer),
        .led_alarm(led_alarm_timer),
        .led_start(led_start_timer),
        .buzz(buzz_timer)
    );
    
    // STOP_WATCH
    stop_watch_ms_top stopwatch_instance (
        .clk(clk),
        .reset_p(reset_p),
        .btn(stop_watch_btn),
        .com(com_stopwatch),
        .seg_7(seg_7_stopwatch),
        .led_start(led_start_stopwatch),
        .led_lap(led_lap_stopwatch)
    );
    
    // WATCH
     loadable_watch_top watch_instance (
        .clk(clk),
        .reset_p(reset_p),
        .btn(watch_btn),
        .com(com_watch),
        .seg_7(seg_7_watch)
    );
    
    //  PARAMETER
    parameter S_TIMER = 3'b001;
    parameter S_STOPWATCH = 3'b010;
    parameter S_WATCH = 3'b100;
    
    reg [2:0] state, next_state; // 상태 변경
    
    // State machine
    always @(negedge clk or posedge reset_p)begin
        if(reset_p)begin
            state = S_TIMER; //리셋시 초기 상태를 타이머로 설정 
        end
        else begin
            state = next_state; // 다음 상태로 이동
        end
    end
    
    // 상태 결정 및 버튼 입력 처리
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            next_state = S_TIMER;  // 초기 상태는 타이머로 설정
        end 
        else if (btn_mode) begin  // 버튼 눌림 감지 
            case (state)
                S_TIMER: next_state = S_STOPWATCH;
                S_STOPWATCH: next_state = S_WATCH;
                S_WATCH: next_state = S_TIMER;
                default: next_state = S_TIMER;
            endcase
        end 
        else begin
            next_state = state;  // 버튼이 눌리지 않은 경우 상태 유지
        end
    end
    
   // 상태에 따라 모듈의 출력 선택
    always @(*) begin
        case (state)
            S_WATCH: begin
                com = com_watch;
                seg_7 = seg_7_watch;
                led_start = 1'b0; // START LED 초기화 
                led_lap = 1'b0; // LAP LED 초기화 
                led_alarm = led_alarm_timer; 
                buzz = buzz_timer; 
                watch_btn = btn;
                stop_watch_btn = 0;
                cooktimer_btn = 0;
            end
            S_STOPWATCH: begin
                com = com_stopwatch;
                seg_7 = seg_7_stopwatch;
                led_start = led_start_stopwatch;
                led_lap = led_lap_stopwatch;
                led_alarm = led_alarm_timer;
                buzz = buzz_timer;
                watch_btn = 0;
                stop_watch_btn = btn;
                cooktimer_btn = 0;
            end
            S_TIMER: begin
                com = com_timer;
                seg_7 = seg_7_timer;
                led_start = led_start;
                led_alarm = led_alarm_timer;
                buzz = buzz_timer;
                watch_btn = 0;
                stop_watch_btn = 0;
                cooktimer_btn = btn;
            end
            default: begin
                com = com_timer;
                seg_7 = seg_7_timer;
                led_start = led_start;
                led_alarm = led_alarm_timer;
                buzz = buzz_timer;
                watch_btn = 0;
                stop_watch_btn = 0;
                cooktimer_btn = btn;
            end
        endcase
    end 
endmodule

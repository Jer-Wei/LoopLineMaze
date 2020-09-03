function detect_crossroad_type () {
    IR_L = 0
    IR_R = 0
    while (true) {
        IR_new = get_IR_Data()
        if (IR_new[2] > 1200) {
            IR_M = 1
        } else {
            IR_M = 0
        }
        if (IR_new[0] > 1200) {
            IR_L = 1
        }
        if (IR_new[4] > 1200) {
            IR_R = 1
        }
        // 終點圓形判定
        // IR[0], IR[4], IR[2] 均為 1200以上50次
        if (IR_new[0] > 1200 && IR_new[4] > 1200 && IR_new[2] > 1200) {
            goal_counter += 1
            if (goal_counter > 50) {
                goal_counter = 0
                // 終點(8)
                crossroad_type = 8
                break;
            }
        } else {
            goal_counter = 0
        }
        // 十字路口或T字路口判斷
        if (IR_new[0] < 500 && IR_L == 1 && (IR_new[4] < 500 && IR_R == 1)) {
            if (IR_M == 1) {
                // 十字路口 (7)
                crossroad_type = 7
                break;
            } else {
                // T字路口 (3)
                crossroad_type = 3
                break;
            }
        }
        // 左卜或左彎判斷
        if (IR_new[0] < 500 && IR_L == 1 && IR_R == 0) {
            if (IR_M == 1) {
                // 左卜(5)
                crossroad_type = 5
                break;
            } else {
                // 左彎 (1)
                crossroad_type = 1
                break;
            }
        }
        // 右卜或右彎判斷
        if (IR_new[4] < 500 && IR_R == 1 && IR_L == 0) {
            if (IR_M == 1) {
                // 右卜(6)
                crossroad_type = 6
                break;
            } else {
                // 右彎(2)
                crossroad_type = 2
                break;
            }
        }
        // 死路判斷
        if (IR_new[2] < 500 && (IR_new[4] < 500 && IR_new[0] < 500) && (IR_L == 0 && IR_R < 0)) {
            // 右彎(2)
            crossroad_type = 4
            break;
        }
    }
}
// 追蹤並跟隨直線前進
function trace_line (speed: number, Kp: number, Kd: number) {
    line_position = BitRacer.readLine()
    trace_err = 0 - line_position
    delta_err = trace_err - trace_err_old
    trace_err_old = trace_err
    PD_Value = Kp * trace_err + Kd * delta_err
    BitRacer.motorRun(BitRacer.Motors.M_R, speed - PD_Value)
    BitRacer.motorRun(BitRacer.Motors.M_L, speed + PD_Value)
}
input.onButtonPressed(Button.A, function () {
    BitRacer.motorRun(BitRacer.Motors.All, 0)
    ModeSelected += 1
    if (ModeSelected > 5) {
        ModeSelected = 0
    }
    basic.showNumber(ModeSelected)
})
function discoverTreeMaze () {
    drive_car(0)
}
// 車輛駕駛(模式)
// 0:直行, 1:左轉, 2:右轉, 3:迴轉, 4:停止
function drive_car (Mode: number) {
    trace_err_old = 0
    speed_trun = 390
    // 直行
    if (Mode == 0) {
        basic.showString("S")
        while (true) {
            trace_line(350, 250, 140)
            IR_new = get_IR_Data()
            line_counter += 1
            if (line_counter > 40 && (IR_new[0] > 1200 || IR_new[4] > 1200 || IR_new[1] < 500 && IR_new[2] < 500 && IR_new[3] < 500)) {
                line_counter = 0
                BitRacer.motorRun(BitRacer.Motors.All, 50)
                break;
            }
        }
        basic.clearScreen()
        detect_crossroad_type()
    }
    // 左轉
    if (Mode == 1) {
        BitRacer.LED(BitRacer.LEDs.LED_L, BitRacer.LEDswitch.on)
        basic.showString("L")
        while (true) {
            BitRacer.motorRun(BitRacer.Motors.M_L, 0 + speed_trun)
            BitRacer.motorRun(BitRacer.Motors.M_R, 0 - speed_trun)
            IR_new = get_IR_Data()
            turn_counter += 1
            if (turn_counter > 100 && IR_new[0] > 1200) {
                turn_counter = 0
                while (true) {
                    trace_line(0, 220, 250)
                    if (line_position >= -0.3 && line_position <= 0.3) {
                        BitRacer.motorRun(BitRacer.Motors.All, 0)
                        break;
                    }
                }
                break;
            }
        }
        BitRacer.LED(BitRacer.LEDs.LED_L, BitRacer.LEDswitch.off)
        basic.clearScreen()
    }
    if (Mode == 2) {
        BitRacer.LED(BitRacer.LEDs.LED_R, BitRacer.LEDswitch.on)
        basic.showString("R")
        while (true) {
            BitRacer.motorRun(BitRacer.Motors.M_L, 0 - speed_trun)
            BitRacer.motorRun(BitRacer.Motors.M_R, 0 + speed_trun)
            IR_new = get_IR_Data()
            turn_counter += 1
            if (turn_counter > 100 && IR_new[4] > 1200) {
                turn_counter = 0
                while (true) {
                    trace_line(0, 220, 250)
                    if (line_position >= -0.3 && line_position <= 0.3) {
                        BitRacer.motorRun(BitRacer.Motors.All, 0)
                        break;
                    }
                }
                break;
            }
        }
        BitRacer.LED(BitRacer.LEDs.LED_R, BitRacer.LEDswitch.off)
        basic.clearScreen()
    }
    if (Mode == 3) {
        BitRacer.LED(BitRacer.LEDs.LED_R, BitRacer.LEDswitch.on)
        basic.showString("R")
        while (true) {
            BitRacer.motorRun(BitRacer.Motors.M_L, 0 - speed_trun)
            BitRacer.motorRun(BitRacer.Motors.M_R, 0 + speed_trun)
            IR_new = get_IR_Data()
            turn_counter += 1
            if (turn_counter > 200 && IR_new[4] > 1200) {
                turn_counter = 0
                while (true) {
                    trace_line(0, 220, 250)
                    if (line_position >= -0.3 && line_position <= 0.3) {
                        BitRacer.motorRun(BitRacer.Motors.All, 0)
                        break;
                    }
                }
                break;
            }
        }
        BitRacer.LED(BitRacer.LEDs.LED_R, BitRacer.LEDswitch.off)
        basic.clearScreen()
    }
    if (Mode == 4) {
        BitRacer.motorRun(BitRacer.Motors.All, 0)
    }
}
input.onButtonPressed(Button.AB, function () {
    BitRacer.motorRun(BitRacer.Motors.All, 0)
})
input.onButtonPressed(Button.B, function () {
    if (ModeSelected == 0) {
        basic.pause(1000)
        CalibrateIR()
        music.playMelody("C C E - C5 C5 - - ", 120)
    }
    if (ModeSelected == 1) {
        discoverTreeMaze()
    }
    if (ModeSelected == 2) {
    	
    }
    if (ModeSelected == 3) {
    	
    }
    if (ModeSelected == 4) {
    	
    }
    if (ModeSelected == 5) {
        BitRacer.motorRun(BitRacer.Motors.All, 900)
        basic.pause(5000)
        BitRacer.motorRun(BitRacer.Motors.All, -900)
        basic.pause(5000)
        BitRacer.motorRun(BitRacer.Motors.All, 0)
    }
})
// 校正紅外線感應器的特性曲線
function CalibrateIR () {
    BitRacer.CalibrateBegin()
    BitRacer.motorRun(BitRacer.Motors.All, 200)
    basic.pause(500)
    BitRacer.motorRun(BitRacer.Motors.All, 0)
    BitRacer.CalibrateEnd(BitRacer.LineColor.White)
}
// 讀取IR偵測器的數值
function get_IR_Data () {
    IR = []
    for (let IR_no = 0; IR_no <= 4; IR_no++) {
        IR[IR_no] = BitRacer.readIR2(IR_no)
    }
    return IR
}
let IR: number[] = []
let turn_counter = 0
let line_counter = 0
let speed_trun = 0
let PD_Value = 0
let trace_err_old = 0
let delta_err = 0
let trace_err = 0
let line_position = 0
let crossroad_type = 0
let goal_counter = 0
let IR_M = 0
let IR_new: number[] = []
let IR_R = 0
let IR_L = 0
let ModeSelected = 0
BitRacer.motorRun(BitRacer.Motors.All, 0)
ModeSelected = 0
basic.showNumber(ModeSelected)

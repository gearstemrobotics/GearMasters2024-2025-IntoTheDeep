package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class NewTestAutoEncode extends BaseAuto {


    @Override
    public void RunOpModeInnerLoop() {
        int hex_motor_ticks = 288;
        arm(0, hex_motor_ticks * 12, 0, 0.5);
        drive(-hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, 0.5);
        drive(-hex_motor_ticks * 8, hex_motor_ticks * 8, -hex_motor_ticks * 8, hex_motor_ticks * 8, 0.5);
        arm(0, -hex_motor_ticks * 12, 0, 0.5);
        // Moves to the wall
        sleep(3000);
        //Moves to the corner
        drive(hex_motor_ticks * 7, -hex_motor_ticks * 7, hex_motor_ticks * 7, -hex_motor_ticks * 7, 0.5);

    }
}
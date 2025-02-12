
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestPickupAuto extends BaseAuto {


    @Override
    public void RunOpModeInnerLoop() {

        int hex_motor_ticks = 288;
        // arm(0, 0, -hex_motor_ticks * 1.7, 0.5);
        arm(hex_motor_ticks * 8, 0, -hex_motor_ticks * 1.7, 1 );
        drive(hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, 1);
        //place it in the basket
        gripper2.setPower(-1);
        sleep(2500);
        gripper2.setPower(0);
        //start moving the arm back to position
        arm(0, 0, -hex_motor_ticks * 0.5, 0.5);
        drive(-hex_motor_ticks * 2.7, -hex_motor_ticks * 2.7, -hex_motor_ticks * 2.7, -hex_motor_ticks * 2.7, 0.5);
        //arm(0, 0, hex_motor_ticks * 1, 0.5);
        arm(-hex_motor_ticks * 9, 0, hex_motor_ticks * 1, 1);



        //drive forward
        drive(-hex_motor_ticks * 4, hex_motor_ticks * 4, hex_motor_ticks * 4, -hex_motor_ticks * 4, 0.75);
        //turn
        drive(-hex_motor_ticks * 4, -hex_motor_ticks * 4, hex_motor_ticks * 4, hex_motor_ticks * 4, 0.75);
        //drive to corner
        drive(hex_motor_ticks * 10, -hex_motor_ticks * 10, -hex_motor_ticks * 10, hex_motor_ticks * 10, 0.75);

        gripper2.setPower(1);
        //5.8
       // arm(hex_motor_ticks * 2.25, 0, 0, 1);
        arm(hex_motor_ticks * 2.25, 0, hex_motor_ticks * 4, 1);
        sleep(500);
        drive(hex_motor_ticks * 1, hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, 1);
        drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, hex_motor_ticks * 1, hex_motor_ticks * 1, 1);
        drive(hex_motor_ticks * 1, hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, 1);
        drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, hex_motor_ticks * 1, hex_motor_ticks * 1, 1);
        sleep(3000);
        gripper2.setPower(0);

    }
}
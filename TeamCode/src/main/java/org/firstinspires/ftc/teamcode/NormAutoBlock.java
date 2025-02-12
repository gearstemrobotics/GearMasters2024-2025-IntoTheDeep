
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PutsTheThingInTheBasket")
public class NormAutoBlock extends BaseAuto {


    @Override
    public void RunOpModeInnerLoop() {

        int hex_motor_ticks = 288;
        arm(hex_motor_ticks * 2, 0, 0, 0.5);
        arm(0, 0, -hex_motor_ticks * 1.5, 0.5);
        arm(hex_motor_ticks * 8, 0, 0, 0.5);
        drive(hex_motor_ticks * 0.5, hex_motor_ticks * 0.5, -hex_motor_ticks * 0.5, -hex_motor_ticks * 0.5, 0.5);
         drive(hex_motor_ticks * 3, hex_motor_ticks * 3, hex_motor_ticks * 3, hex_motor_ticks * 3, 0.5);

         gripper2.setPower(1);
         gripper.setPower(-1);
         sleep(3000);
         gripper2.setPower(0);
        gripper.setPower(0);

        //next phase

    }
}

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BasketPlusPushBlocks extends BaseAuto {


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
         sleep(6000);
         gripper2.setPower(0);
        gripper.setPower(0);



        // push phase

        arm(0, 0, -hex_motor_ticks * 0.7, 0.5);
        drive(-hex_motor_ticks * 3, -hex_motor_ticks * 3, -hex_motor_ticks * 3, -hex_motor_ticks * 3, 0.5);
        arm(0, 0, hex_motor_ticks * 1, 0.5);
        arm(-hex_motor_ticks * 9, 0, 0, 0.5);
        //strafe
        drive(-hex_motor_ticks * 11, hex_motor_ticks * 11, hex_motor_ticks * 11, -hex_motor_ticks * 11, 0.3);
        // moves forward
        drive(hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.3);
        drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, hex_motor_ticks * 1, hex_motor_ticks * 1, 0.3);

        // half delivers
        drive(hex_motor_ticks * 11, -hex_motor_ticks * 11, -hex_motor_ticks * 11, hex_motor_ticks * 11, 0.3);

        //Turns a little right
        //  drive(-hex_motor_ticks * 1.3, -hex_motor_ticks * 1.3, hex_motor_ticks * 1.3, hex_motor_ticks * 1.3, 0.3);

        //delivers
        // drive(hex_motor_ticks * 3, -hex_motor_ticks * 3, -hex_motor_ticks * 3, hex_motor_ticks * 3, 0.3);

        //bring arm down reset
        // arm(0, -hex_motor_ticks * 11, 0, 0.5);

        //strafe right
        drive(-hex_motor_ticks * 10, hex_motor_ticks * 10, hex_motor_ticks * 10, -hex_motor_ticks * 10 , 0.3);

        //moves forward a little
        drive(hex_motor_ticks *2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks *2, 0.3);

        //delivers
        drive(hex_motor_ticks * 10, -hex_motor_ticks * 10, -hex_motor_ticks *10, hex_motor_ticks * 10, 0.5);

        //strafe right for third block
        drive(-hex_motor_ticks * 10, hex_motor_ticks * 10, hex_motor_ticks * 10, -hex_motor_ticks * 10  , 0.3);

        //aligns with the block 3 move forward
        drive(hex_motor_ticks *2.3, hex_motor_ticks * 2.3, hex_motor_ticks * 2.3, hex_motor_ticks *2.3, 0.3);

        //delivers
        drive(hex_motor_ticks * 11, -hex_motor_ticks * 11, -hex_motor_ticks *11, hex_motor_ticks * 11, 1);

    }
}
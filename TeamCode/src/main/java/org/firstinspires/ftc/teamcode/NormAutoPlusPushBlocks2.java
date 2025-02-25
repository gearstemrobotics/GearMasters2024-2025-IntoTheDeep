

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class NormAutoPlusPushBlocks2 extends BaseAuto {


    @Override
    public void RunOpModeInnerLoop() {
        int hex_motor_ticks = 288;
        // arm(0, hex_motor_ticks * 12, 0, 0.7);

        //drive(-hex_motor_ticks * 9 , hex_motor_ticks * 9, hex_motor_ticks * 9, -hex_motor_ticks * 9, 0.7);
        //Drive + arm  at same time
        everything(-hex_motor_ticks * 9,hex_motor_ticks * 9,hex_motor_ticks * 9,-hex_motor_ticks * 9,0,hex_motor_ticks * 12,0, 1,0.4);
        arm(0, -hex_motor_ticks * 11, 0, 1);
        // Moves to the wall
        //Moves back a little bit
        drive(hex_motor_ticks * 3.7, -hex_motor_ticks * 3.7, -hex_motor_ticks * 3.7, hex_motor_ticks * 3.7, 0.3);

        //Moves a little bit towards the blocks
        drive(hex_motor_ticks * 6.9, hex_motor_ticks * 6.9, hex_motor_ticks * 6.9, hex_motor_ticks * 6.9, 0.3);

        //strafes right
        drive(-hex_motor_ticks * 8, hex_motor_ticks * 8, hex_motor_ticks * 8, -hex_motor_ticks * 8, 0.3);

        // moves forward
        drive(hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.3);

        // half delivers
        drive(hex_motor_ticks * 9.5, -hex_motor_ticks * 9.5, -hex_motor_ticks * 9.5, hex_motor_ticks * 9.5, 0.3);

        //Turns a little right
      //  drive(-hex_motor_ticks * 1.3, -hex_motor_ticks * 1.3, hex_motor_ticks * 1.3, hex_motor_ticks * 1.3, 0.3);

        //delivers
       // drive(hex_motor_ticks * 3, -hex_motor_ticks * 3, -hex_motor_ticks * 3, hex_motor_ticks * 3, 0.3);

        //bring arm down reset
       // arm(0, -hex_motor_ticks * 11, 0, 0.5);

        //strafe right
        drive(-hex_motor_ticks * 8, hex_motor_ticks * 8, hex_motor_ticks * 8, -hex_motor_ticks * 8 , 0.3);


        //moves forward a little
        drive(hex_motor_ticks *2.5, hex_motor_ticks * 2.5, hex_motor_ticks * 2.5, hex_motor_ticks *2.5, 0.3);

        //delivers
        drive(hex_motor_ticks * 9, -hex_motor_ticks * 9, -hex_motor_ticks *9, hex_motor_ticks * 9, 0.5);

        //strafe right for third block
        drive(-hex_motor_ticks * 9, hex_motor_ticks * 9, hex_motor_ticks * 9, -hex_motor_ticks * 9 , 0.3);

        //aligns with the block 3 move forward
        drive(hex_motor_ticks *2.5, hex_motor_ticks * 2.5, hex_motor_ticks * 2.5, hex_motor_ticks *2.5, 0.3);

        //delivers
        drive(hex_motor_ticks * 11, -hex_motor_ticks * 11, -hex_motor_ticks *11, hex_motor_ticks * 11, 1);
    }
}
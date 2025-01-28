
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class NormAutoPlusPushBlocks extends BaseAuto {


    @Override
    public void RunOpModeInnerLoop() {
        int hex_motor_ticks = 288;
       // arm(0, hex_motor_ticks * 12, 0, 0.7);

        //drive(-hex_motor_ticks * 9 , hex_motor_ticks * 9, hex_motor_ticks * 9, -hex_motor_ticks * 9, 0.7);
        //Drive + arm  at same time
        arm(0, -hex_motor_ticks * 11, 0, 1);
        // Moves to the wall
        everything(-hex_motor_ticks * 9,hex_motor_ticks * 9,hex_motor_ticks * 9,-hex_motor_ticks * 9,0,-hex_motor_ticks * 11,0, 0.7);
        //Moves back a little bit
        drive(hex_motor_ticks * 3.5, -hex_motor_ticks * 3.5, -hex_motor_ticks * 3.5, hex_motor_ticks * 7, 0.3);
        //Moves a little bit towards the blocks
        drive(hex_motor_ticks * 7.3, hex_motor_ticks * 7.3, hex_motor_ticks * 7.3, hex_motor_ticks * 7.3, 0.3);
        //strafes right
        drive(-hex_motor_ticks * 10.5, hex_motor_ticks * 10.5, hex_motor_ticks * 10.5, -hex_motor_ticks * 10.5, 0.3);
        // moves forward
        drive(hex_motor_ticks * 4.5, hex_motor_ticks * 4.5, hex_motor_ticks * 4.5, hex_motor_ticks * 4.5, 0.3);
        // half delivers
        drive(hex_motor_ticks * 9.5, -hex_motor_ticks * 9.5, -hex_motor_ticks * 9.5, hex_motor_ticks * 9.5, 0.3);
        //Turns a little right
        drive(-hex_motor_ticks * 1.3, -hex_motor_ticks * 1.3, hex_motor_ticks * 1.3, hex_motor_ticks * 1.3, 0.3);

        //delivers
        drive(hex_motor_ticks * 5.5, -hex_motor_ticks * 5.5, -hex_motor_ticks * 5.5, hex_motor_ticks * 5.5, 1);
        //bring arm down reset
        arm(0, -hex_motor_ticks * 11, 0, 0.5);
    }
}
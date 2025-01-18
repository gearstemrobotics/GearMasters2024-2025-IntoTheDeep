
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "JustPutsTheThingOnTheThing")
public class NormAutoBlock extends BaseAuto {


    @Override
    public void RunOpModeInnerLoop() {
        int hex_motor_ticks = 288;
       // arm(0, hex_motor_ticks * 12, 0, 0.5);

        //strafe right
       // drive(-hex_motor_ticks * 8, hex_motor_ticks * 8, hex_motor_ticks * 8, -hex_motor_ticks * 8, 0.5);
       // arm(0, -hex_motor_ticks * 12, 0, 0.5);
        // Moves to the wall
        //Moves to the corner
        //drive(hex_motor_ticks * 6, -hex_motor_ticks * 6, -hex_motor_ticks * 6, hex_motor_ticks * 6, 0.5);
        //drive(hex_motor_ticks * 6, hex_motor_ticks * 6, hex_motor_ticks * 6, hex_motor_ticks * 6, 0.5);
        //drive(-hex_motor_ticks * 6, -hex_motor_ticks * 6, hex_motor_ticks * 6, hex_motor_ticks * 6, 0.5);
        //arm(0, 0, hex_motor_ticks * 2, 0.5);
        //drive(hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.5);
         gripper2.setPower(1);
    }
}
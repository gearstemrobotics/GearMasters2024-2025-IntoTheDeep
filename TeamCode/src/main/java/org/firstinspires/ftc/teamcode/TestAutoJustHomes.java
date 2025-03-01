package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestAutoJustHomes extends BaseAuto {

    @Override
    protected void RunInit()
    {

        super.RunInit();
    }

    @Override
    public void RunOpModeInnerLoop() {

        int hex_motor_ticks = 288;
        // arm(0, hex_motor_ticks * 12, 0, 0.7);
        Home();
        //drive(-hex_motor_ticks * 9 , -hex_motor_ticks * 9, hex_motor_ticks * 9, hex_motor_ticks * 9, 1);
    }
}
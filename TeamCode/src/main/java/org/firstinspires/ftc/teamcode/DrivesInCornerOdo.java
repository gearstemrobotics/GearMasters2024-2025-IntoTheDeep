package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.EncoderMacrosForOdoAuto.Operation;

@Autonomous(name = "1DrivesInCornerOdo")
public class DrivesInCornerOdo extends BaseOdoAuto{

    @Override
    public void RunOpModeInnerLoop()
    {
        EncoderMacrosForOdoAuto macro = super.EncoderMacrosForOdoAutoTask;
        double Tick = 100;
        move(30,0,0);
    }
}

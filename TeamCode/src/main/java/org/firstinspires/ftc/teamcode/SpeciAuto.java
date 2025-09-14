package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.EncoderMacrosForOdoAuto.Operation;

@Autonomous(name = "1SpeciAuto")
public class SpeciAuto extends BaseOdoAuto{

    @Override
    public void RunOpModeInnerLoop()
    {
        EncoderMacrosForOdoAuto macro = super.EncoderMacrosForOdoAutoTask;
        double Tick = 100;

        macro.DoOperation(Operation.MoveArmDownSpeci);
        move(-28,0,0);
        macro.CompleteOperation();
        macro.DoOperation(Operation.MoveArmDown);
        macro.CompleteOperation();
        move(0,30 ,0);

    }
}
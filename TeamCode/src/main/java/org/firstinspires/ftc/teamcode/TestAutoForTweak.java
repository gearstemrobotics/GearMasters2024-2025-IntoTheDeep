package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.EncoderMacrosForOdoAuto.Operation;

@Autonomous(name = "1TestAutoForTweak")
public class TestAutoForTweak extends BaseOdoAuto{

    @Override
    public void RunOpModeInnerLoop()
    {
        EncoderMacrosForOdoAuto macro = super.EncoderMacrosForOdoAutoTask;
        double Tick = 100;

        // begin the move arm in operation
        macro.DoOperation(Operation.MoveArmIn);

        // give the arm some time to get out of the way before we move
        sleep(250);

        // move the robot around...
        move(30,0,0);

        // wait for operation to stop
        macro.CompleteOperation();

//move(10,0,0);
        //Home();

       // FrontLeft.getCurrentPosition();
         /*
        while (opModeIsActive()) {
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("FrontLeft", FrontLeft.getCurrentPosition());
            telemetry.addData("FrontRight", FrontRight.getCurrentPosition());
            telemetry.addData("BackRight", BackRight.getCurrentPosition());
            telemetry.addData("BackLeft", BackLeft.getCurrentPosition());
            // drive(Tick * 1,Tick * 1, Tick * 1,Tick * 1,0.5);
            telemetry.update();
            }
          */
        }
    }


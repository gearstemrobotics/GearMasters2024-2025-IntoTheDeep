package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "1TestAutoForTweak")
public class TestAutoForTweak extends BaseOdoAuto{

    @Override
    public void RunOpModeInnerLoop()
    {

double Tick = 100;
//move(10,0,0);
        Home();
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


package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Autonomous
public class TestTouch extends LinearOpMode {

    private TouchSensor touch;
    private TouchSensor touch2;
    @Override public void runOpMode(){
        touch = hardwareMap.get(TouchSensor.class,"touch");
        touch2 = hardwareMap.get(TouchSensor.class,"touch2");

        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive()) {

                boolean touched;

                if (touch.isPressed()||touch2.isPressed())
                {
                    touched = true;
                }
                else
                {
                    touched = false;
                }

                 /*
                if (!touch.isPressed())
                {
                    touched = false;
                }
                else
                {
                    touched = true;
                }

                  */

                telemetry.addData("pressed", touched);
                telemetry.update();

            }

        }
    }
}

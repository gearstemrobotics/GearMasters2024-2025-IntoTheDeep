package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TestColorSensor extends LinearOpMode {

    private ColorSensor color;
    @Override public void runOpMode(){

        color = hardwareMap.get(ColorSensor.class, "Color");
        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();

            }

        }
    }
}

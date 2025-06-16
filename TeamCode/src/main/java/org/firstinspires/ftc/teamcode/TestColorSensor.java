package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class TestColorSensor extends LinearOpMode {

    private ColorRangeSensor color;
    private TouchSensor touch2;
   private boolean touched;
    @Override public void runOpMode(){

        color = hardwareMap.get(ColorRangeSensor.class, "Color");
        touch2 = hardwareMap.get(TouchSensor.class, "touch2");
        waitForStart();
        while (opModeIsActive())
        {
            if (touch2.isPressed())
            {
                touched = true;
            }
            else
            {
                touched = false;
            }
            {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.addData("Range", color.getDistance(DistanceUnit.INCH));
                telemetry.addData("IsTouched", touched);

                telemetry.update();

            }

        }
    }
}

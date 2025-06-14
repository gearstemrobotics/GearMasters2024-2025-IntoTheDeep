package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class TestColorSensor extends LinearOpMode {

    private ColorRangeSensor color;
    @Override public void runOpMode(){

        color = hardwareMap.get(ColorRangeSensor.class, "Color");
        waitForStart();
        if (opModeIsActive())
        {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.addData("Range", color.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }
    }
}

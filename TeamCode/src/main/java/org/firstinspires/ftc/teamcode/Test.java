package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() {


        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {

                telemetry.addData("Test", "Test");
                telemetry.update();
            }
        }
    }
}


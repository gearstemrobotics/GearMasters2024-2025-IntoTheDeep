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
public class JustPutsTheThingOnTheThing extends BaseAuto {


    @Override
    public void RunOpModeInnerLoop() {


        sleep(500);
        int hex_motor_ticks = 288;
        extendPos = 0;
        liftPos = 0;
        anglePos = 0;
        FrontRightPos = 0;
        BackRightPos = 0;
        FrontLeftPos = 0;
        BackLeftPos = 0;
        arm(0, hex_motor_ticks * 12, 0, 1);
        drive(-hex_motor_ticks * 8, hex_motor_ticks * 8, -hex_motor_ticks * 8, hex_motor_ticks * 8, 0.5);
        // moves the arm down
        arm(0, -hex_motor_ticks * 12, 0, 0.5);
        // Moves to the wall
        sleep(200);
        //Moves to the corner
        drive(hex_motor_ticks * 5, -hex_motor_ticks * 5, hex_motor_ticks * 5, -hex_motor_ticks * 5, 0.5);
        drive(-hex_motor_ticks * 7, -hex_motor_ticks * 7, -hex_motor_ticks * 7, -hex_motor_ticks * 7, 0.5);
    }
    //telemetry.addData("April Tags", cameraMonitor.GetIdsFound());
    //telemetry.addData("BackLeft.getCurrentPosition", BackLeft.getCurrentPosition());
    // telemetry.update();
}


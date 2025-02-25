package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Autonomous()
public class CameraTest extends BaseAuto {
    public CameraTest() {
    }

    @Override
    protected void RunOpModeInnerLoop() {
        Home();
    }

}


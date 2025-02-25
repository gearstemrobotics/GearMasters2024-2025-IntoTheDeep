package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Autonomous()
public class CameraTest extends BaseAuto {
    private CameraMonitor cameraMonitor = null;
    public CameraTest() {
    }

    @Override
    protected void RunInit()
    {
        cameraMonitor = new CameraMonitor(webcamName);
        Thread t1 = new Thread(cameraMonitor, "t1");
        t1.start();
    }

    @Override
    protected void RunOpModeInnerLoop() {

        while (opModeIsActive())
        {
            AddTelemtry();
        }
    }

    int LoopCount = 0;

    private void AddTelemtry() {
        telemetry.addData("x(calc)", String.format("%.2f", cameraMonitor.GetCalculatedX()));
        telemetry.addData("x", String.format("%.2f", cameraMonitor.GetX()));
        telemetry.addData("y", String.format("%.2f", cameraMonitor.GetY()));
        telemetry.addData("z", String.format("%.2f", cameraMonitor.GetZ()));
        telemetry.addData("yaw", String.format("%.2f", cameraMonitor.GetYaw()));
        telemetry.addData("range", String.format("%.2f", cameraMonitor.GetRange()));
        telemetry.addData("bearing", String.format("%.2f", cameraMonitor.GetBearing()));
        telemetry.addData("LoopCount", LoopCount);
        telemetry.update();
    }

}


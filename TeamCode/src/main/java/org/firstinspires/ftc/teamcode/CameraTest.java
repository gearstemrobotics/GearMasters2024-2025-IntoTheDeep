package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.CameraRenderer;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;


@Autonomous()
public class CameraTest extends OpMode {

    private CameraMonitor cameraMonitor;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        cameraMonitor = new CameraMonitor(webcamName);
        Thread t1 = new Thread(cameraMonitor,"t1");
        t1.start();
    }

    int LoopCount = 0;

    @Override
    public void init_loop() {
        LoopCount++;

        telemetry.addData("April Tags", cameraMonitor.GetIdsFound());
        telemetry.addData("LoopCount", LoopCount);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        LoopCount++;

        telemetry.addData("April Tags", cameraMonitor.GetIdsFound());
        telemetry.addData("LoopCount", LoopCount);
    }
}

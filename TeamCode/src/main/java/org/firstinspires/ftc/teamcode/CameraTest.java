package org.firstinspires.ftc.teamcode;
//  package org.firstinspires.ftc.teamcode.processors;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
 import org.firstinspires.ftc.vision.VisionProcessor;
 import org.opencv.core.Mat;
import org.opencv.core.Rect;


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
        Thread t1 = new Thread(cameraMonitor, "t1");
        t1.start();
    }

    int LoopCount = 0;

    @Override
    public void init_loop() {
        LoopCount++;

        AddTelemtry();
    }

    private void AddTelemtry() {
        telemetry.addData("x", String.format("%.2f", cameraMonitor.GetX()));
        telemetry.addData("y", String.format("%.2f", cameraMonitor.GetY()));
        telemetry.addData("z", String.format("%.2f", cameraMonitor.GetZ()));
        telemetry.addData("roll", String.format("%.2f", cameraMonitor.GetRoll()));
        telemetry.addData("pitch", String.format("%.2f", cameraMonitor.GetPitch()));
        telemetry.addData("yaw", String.format("%.2f", cameraMonitor.GetYaw()));
        telemetry.addData("LoopCount", LoopCount);
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        LoopCount++;
       // if (cameraMonitor.GetX() < 0){}
        AddTelemtry();
    }
}
//
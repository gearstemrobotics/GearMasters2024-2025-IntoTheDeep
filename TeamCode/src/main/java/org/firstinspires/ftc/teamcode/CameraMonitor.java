package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;


import android.os.Build;
import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import java.util.List;

public class CameraMonitor implements Runnable {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private boolean isRunning = true;

    private StringBuilder lastIdsFound = new StringBuilder();



    public CameraMonitor(WebcamName webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    @Override
    public void run() {

        while (isRunning) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            StringBuilder idsFound = new StringBuilder();



            if (currentDetections.size() > 0)
            {
                AprilTagDetection d = currentDetections.get(0);
                _x = CalculateX(d.ftcPose.y, d.ftcPose.yaw);
                _y = d.ftcPose.y;
                _z = d.ftcPose.z;
                _roll = d.ftcPose.roll;
                _pitch = d.ftcPose.pitch;
                _yaw = d.ftcPose.yaw;
                idsFound.append(String.format("%s",d.id));
            }
            else
            {
                _x = Double.NaN;
                _y = Double.NaN;
                _z = Double.NaN;
                _roll = Double.NaN;
                _pitch= Double.NaN;
                _yaw =  Double.NaN;
                idsFound.append("");
            }


            lastIdsFound = idsFound;
        }
    }

    private double CalculateX(double y, double yaw) {
        // x is so wrong, calculate instead
        return Math.tan(Math.toRadians(yaw)) * y;
    }

    public StringBuilder GetIdsFound() {
        return lastIdsFound;
    }

    volatile double _x = Double.NaN;
    volatile double _y = Double.NaN;
    volatile double _z = Double.NaN;
    volatile double _roll = Double.NaN;
    volatile double _pitch = Double.NaN;
    volatile double _yaw = Double.NaN;

    public double GetX(){ return _x; }
    public double GetY(){ return _y; }
    public double GetZ(){ return _z; }
    public double GetRoll(){ return _roll; }
    public double GetPitch(){ return _pitch; }
    public double GetYaw(){ return _yaw; }

    public void stop() {
        isRunning = false;
        visionPortal.stopStreaming();
    }
}

package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
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
import java.util.concurrent.TimeUnit;

public class CameraMonitor implements Runnable {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private boolean isRunning = true;

    private StringBuilder lastIdsFound = new StringBuilder();
    private double fx = 1427.01;
    private double fy = 1427.01;
    private double cx = 615.089;
    private double cy = 344.519;

     public double FrameRate = 0;

    public CameraMonitor(WebcamName webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //.setLensIntrinsics(fx,fy,cx,cy)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(webcamName)
                //.setCameraResolution(new Size(1280, 720))
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();



    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (isRunning && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (isRunning)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void sleep(long ms)
    {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    public void run() {
        setManualExposure(6,250);

        while (isRunning) {


            sleep(10);
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            StringBuilder idsFound = new StringBuilder();



            if (currentDetections.size() > 0)
            {
                AprilTagDetection d = currentDetections.get(0);
                _pose = d.ftcPose;
                _x = CalculateX(d.ftcPose.y, d.ftcPose.yaw);
                FrameRate = visionPortal.getFps();
                idsFound.append(String.format("%s",d.id));
            }
            else
            {
                _pose = null;
                _x = Double.NaN;
                idsFound.append("");
                FrameRate = 0;
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

    public AprilTagPoseFtc _pose = null;
    volatile double _x = Double.NaN;

    public AprilTagPoseFtc GetPose()
    {
        return _pose;
    }

    public double GetCalculatedX(){ return _x; }

    public double GetX()
    {
        if (_pose == null) return Double.NaN;
        return _pose.x;
    }

    public double GetY()
    {
        if (_pose == null) return Double.NaN;
        return _pose.y;
    }

    public double GetZ()
    {
        if (_pose == null) return Double.NaN;
        return _pose.z;
    }

     public double GetYaw(){
         if (_pose == null) return Double.NaN;
         return _pose.yaw;
     }

    public double GetRange(){
        if (_pose == null) return Double.NaN;
        return _pose.range;
    }

    public double GetBearing(){
        if (_pose == null) return Double.NaN;
        return _pose.bearing;
    }

    public void stop() {
        isRunning = false;
        visionPortal.stopStreaming();
    }

    public boolean IsReady()
    {
        return visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;

    }

}

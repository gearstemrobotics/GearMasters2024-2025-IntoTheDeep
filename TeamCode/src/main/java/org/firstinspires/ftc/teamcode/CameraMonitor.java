package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraMonitor implements Runnable {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private boolean isRunning = true;

    private StringBuilder lastIdsFound = new StringBuilder();

    public CameraMonitor(WebcamName webcamName) {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    @Override
    public void run() {
        while (isRunning) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            StringBuilder idsFound = new StringBuilder();
            for (AprilTagDetection detection : currentDetections) {
                idsFound.append(detection.id);
                idsFound.append(' ');
            }

            lastIdsFound = idsFound;
        }
    }

    public StringBuilder GetIdsFound() {
        return lastIdsFound;
    }

    public void stop() {
        isRunning = false;
        visionPortal.stopStreaming();
    }
}

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

import androidx.fragment.app.FragmentContainer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.CameraRenderer;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;

import com.qualcomm.robotcore.hardware.CRServo;


@Autonomous()
public class CameraTest extends OpMode {

    private CameraMonitor cameraMonitor;
    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;

    private DcMotor BackRight;
    private CRServo gripper;


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
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        gripper = gripper = hardwareMap.get(CRServo.class, "gripper");
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
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        //BackLeft.setDirection(DcMotor.Direction.REVERSE);
        LoopCount++;
        // if (cameraMonitor.GetX() < 0){}
        AddTelemtry();
        double Y = cameraMonitor.GetY();
        double X = cameraMonitor.GetX();
        double Yaw = cameraMonitor.GetYaw();
        double power1 = 0;
        double power2 = 0;
        double power3 = 0;
        double seeNoPower = 0;
        boolean isStrafe = false;
        boolean isFor = false;
        boolean isTurn = false;
        String setStatus = "nothing";


        // forward back X
        if (Double.isNaN(X)) {
            power1 = 0;
            isFor = false;
        } else if (X >= 3) {
            //move back
            power1 = 0.3;
            isFor = true;
            setStatus = "Forward";
        } else if (X < -3) {
            // move forward
            power1 = - 0.3;
            isFor = true;
            setStatus = "Back";
        } else {
            power1 = 0;
            isFor = false;
        }

        //Turn Yaw
        if (Double.isNaN(Yaw) || isFor) {
            power2 = 0;
        } else if (Yaw >= 5) {
            power2 = 0.3;
            isTurn = true;
            setStatus = "turn";
        } else if (Yaw <= -5) {
            power2 = -0.3;
            isTurn = true;
            setStatus = "Turn";
        } else {
            power2 = 0;
            isTurn = false;
        }


        //Strafe Y
        if (Double.isNaN(Y) || isFor || isTurn) {
            power3 = 0;
        } else if (Y >= 13) {
            power3 = -0.3;
            isStrafe = true;
            setStatus = "Target";
        } else {
            power3 = 0;

        }

        if (Double.isNaN(Y) && Double.isNaN(X) && Double.isNaN(Yaw)) {
            // zeroout turn and
            seeNoPower = 0.1;
        } else {
            seeNoPower = 0;
        }
        //Power 1 = 1st set For/ Back
        if (isFor) {
            FrontLeft.setPower(power1);
            BackLeft.setPower(power1);
            BackRight.setPower(power1);
            FrontRight.setPower(power1);
        }

        //power 2 = second set turn
        else if (isTurn) {
            FrontLeft.setPower(-power2);
            BackLeft.setPower(-power2);
            BackRight.setPower(power2);
            FrontRight.setPower(power1);
        }

        //power 3 = third set that strafe to target
        else if (isStrafe) {
            FrontLeft.setPower(power3);
            BackLeft.setPower(-power3);
            BackRight.setPower(power3);
            FrontRight.setPower(-power3);
        }

        // seeNoPower = nothing is see just goes in circles
        else {
            FrontLeft.setPower(seeNoPower);
            BackLeft.setPower(seeNoPower);
            BackRight.setPower(-seeNoPower);
            FrontRight.setPower(-seeNoPower);


        }
        telemetry.addData(setStatus,"status");
        telemetry.update();

    }
}
//
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
        boolean isStrafe = false;
        boolean isTurn = false;



        //Strafe X
        if (Double.isNaN(X))
        {
            power2 = 0;
        }
        else if (X >=  13)
        {
            power2 = 0.5;
            isStrafe = true;
        }
        else
        {
            power2 = 0.1;
            isStrafe= true;
        }

        //Turn Yaw
        if (Double.isNaN(Yaw))
        {
            power3 = 0;
        }
        else if (Yaw >=  13)
        {
            power3 = 0.5;
        }
        else
        {
            power3 = 0.1;
        }


        //Forward or Backwards Y
        if (Double.isNaN(Y))
        {
            power1 = 0;
        }
        else if (Y >=  13)
        {
            power1 = 0.5;
        }
        else
        {
            power1 = 0.1;
        }

        if(isStrafe)
        {
            // zeroout turn and

        }

        FrontLeft.setPower(power1);
        // forward/back or y
        gripper.setPower(power2);
        // strafe or x
        BackLeft.setPower(power3);
        // = Turn or yaw
       // BackRight.setPower(power);
        //FrontRight.setPower(power);
    }
}
//
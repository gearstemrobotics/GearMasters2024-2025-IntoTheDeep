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
public class CameraTest extends BaseAuto {

    private CameraMonitor cameraMonitor;
    public CameraTest()
    {
    }

    int LoopCount = 0;

    @Override
    protected void Map()
    {
        super.Map(); // call base class map first


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        cameraMonitor = new CameraMonitor(webcamName);
        Thread t1 = new Thread(cameraMonitor, "t1");
        t1.start();
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
    protected void RunOpModeInnerLoop() {
        // kickoff thread for camera here
        boolean hasJumped = false;

        while (opModeIsActive())
        {
            LoopCount++;
            AddTelemtry();
            telemetry.update();

            try {
                JumpToAprilTag();
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                // ignore this
            }





            /*
            if (AprilTagHoming() && !hasJumped)
            {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                JumpToAprilTag();
                hasJumped = true;
                break;
            }
            */

        }
    }

    /*
    @Override
    public void loop() {
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        //BackLeft.setDirection(DcMotor.Direction.REVERSE);
        LoopCount++;
        // if (cameraMonitor.GetX() < 0){}
        AddTelemtry();

        if (AprilTagHoming())
        {
            JumpToAprilTag();
        }
    }
    */

    private boolean AprilTagHoming() {
        // switch to power mode
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double Y = cameraMonitor.GetY();
        double X = cameraMonitor.GetX();
        double Yaw = cameraMonitor.GetYaw();
        double power1 = 0;
        double power2 = 0;
        double power3 = 0;
        double seeNoPower = 0;
        double Xlimit = 3;
        double Yawlimit = 5;

        boolean isStrafe = false;
        boolean isFor = false;
        boolean isTurn = false;
        String setStatus = "nothing";


        // forward back X
        if (Double.isNaN(X)) {
            power1 = 0;
            isFor = false;
        } else if (X >= Xlimit) {
            //move back
            power1 = 0.3;
            isFor = true;
            setStatus = "Forward";
        } else if (X < -Xlimit) {
            // move forward
            power1 = -0.3;
            isFor = true;
            setStatus = "Back";
        } else {
            power1 = 0;
            isFor = false;
        }

        //Turn Yaw
        if (Double.isNaN(Yaw) || isFor) {
            power2 = 0;
        } else if (Yaw >= Yawlimit) {
            power2 = 0.3;
            isTurn = true;
            setStatus = "turn";
        } else if (Yaw <= -Yawlimit) {
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
      //  } else if (Y <= 11) {
        //    power3 = 0.3;
         //   isStrafe = true;
          //  setStatus = "Target";

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
            FrontRight.setPower(power2);
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
        telemetry.addData(setStatus, "status");
        telemetry.update();
        // try {
        // Thread.sleep(50);
        // } catch (InterruptedException e) {
        //   throw new RuntimeException(e);
        // }

        return !(Double.isNaN(Y) && Double.isNaN(X) && Double.isNaN(Yaw));
    }

    void JumpToAprilTag()
    {
        double Y = cameraMonitor.GetY();
        Y = Y - 13;
        double X = cameraMonitor.GetX();
        double Yaw = cameraMonitor.GetYaw();
        int hex_motor_ticks = 288;
        int turnYaw = (int)(Yaw * -13.5);
        int forX = (int)(X * 60.0);
        int strafeY = (int)(Y * -60.0);
        // rotate by yaw
        drive(-turnYaw * 1,-turnYaw * 1,turnYaw * 1,turnYaw * 1,0.3);

        // move in X
        //drive(hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.3);
        drive(forX * 1,forX * 1,forX * 1,forX * 1,0.3);
        // move in Y

        drive(-strafeY * 1,strafeY * 1,strafeY * 1,-strafeY * 1,0.3);
    }
}
//
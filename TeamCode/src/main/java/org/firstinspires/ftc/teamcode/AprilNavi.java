package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class AprilNavi {
    private BaseAuto baseAuto;
    private CameraMonitor cameraMonitor;

    HomingState state = HomingState.WaitForCamera;

    public enum HomingState {
        WaitForCamera,
        ScanForTag,
        HeadToTag,
        CenterOnTag,
        FineTune,
    }

    public AprilNavi(BaseAuto Homing) {
        baseAuto = Homing;
        cameraMonitor = new CameraMonitor(baseAuto.webcamName);
        Thread t1 = new Thread(cameraMonitor, "t1");
        t1.start();

    }


    int LoopCount = 0;


    private void AddTelemtry() {
        baseAuto.telemetry.addData("state", state.toString());
        baseAuto.telemetry.addData("x(calc)", String.format("%.2f", cameraMonitor.GetCalculatedX()));
        baseAuto.telemetry.addData("x", String.format("%.2f", cameraMonitor.GetX()));
        baseAuto.telemetry.addData("y", String.format("%.2f", cameraMonitor.GetY()));
        baseAuto.telemetry.addData("z", String.format("%.2f", cameraMonitor.GetZ()));
        baseAuto.telemetry.addData("yaw", String.format("%.2f", cameraMonitor.GetYaw()));
        baseAuto.telemetry.addData("range", String.format("%.2f", cameraMonitor.GetRange()));
        baseAuto.telemetry.addData("bearing", String.format("%.2f", cameraMonitor.GetBearing()));
        baseAuto.telemetry.addData("LoopCount", LoopCount);
        baseAuto.telemetry.update();
    }


    public void Home() {
        // kickoff thread for camera here
        boolean hasJumped = false;
        state = HomingState.WaitForCamera;

        while (baseAuto.opModeIsActive()) {
            LoopCount++;
            AddTelemtry();
            baseAuto.telemetry.update();
            if (cameraMonitor.IsReady()) {
                //AprilTagHoming();
                if (state == HomingState.WaitForCamera) {
                    Sleep(2000);
                    state = HomingState.ScanForTag;
                } else if (state == HomingState.ScanForTag) {
                    scanForTag();
                    // wait a bit to make sure tag refreshes
                    Sleep(500);

                    // only stop scanning if we found it
                    if (cameraMonitor.GetPose() != null) {
                        state = HomingState.HeadToTag;
                    }
                } else if (state == HomingState.HeadToTag) {
                    GoToAprilTag();
                    if (cameraMonitor.GetRange() < 17 || Math.abs(cameraMonitor.GetYaw()) > 55) {
                        state = HomingState.CenterOnTag;

                        // wait a bit to make sure tag refreshes
                        Sleep(500);
                    }
                } else if (state == HomingState.CenterOnTag) {
                    JumpToAprilTag();

                    if (Math.abs(cameraMonitor.GetYaw()) < 2) {
                        state = HomingState.FineTune;

                        // wait a bit to make sure tag refreshes
                        Sleep(500);
                    }
                } else if (state == HomingState.FineTune) {
                    AprilTagHoming();

                    if (IsHomed()) {
                        // break out of the loop, we are done
                        break;

                        /*
                        // lets see how we did!
                        Sleep(5000);
                        // rotate 180 and kick off the search again
                        drive(-1000,-1000,1000,10000,0.5);
                        state = HomingState.ScanForTag;
                        */
                    }

                }
            }
        }
    }

    public void Sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
        }
    }

    public boolean IsHomed() {
        AprilTagPoseFtc pose = cameraMonitor.GetPose();
        if (pose != null) {

            double y = pose.y;
            double yaw = Math.abs(pose.yaw);
            if (y < 13 && y > 11 && yaw < 4) {
                return true;
            }
        }
        return false;
    }

    private void scanForTag() {
        int hex_motor_ticks = 56;
        while (true) {
            if (cameraMonitor.GetPose() != null) {
                return;
            }

            for (int i = 0; i < 10; i++) {
                baseAuto.drive(-hex_motor_ticks * 2, -hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.5);
                if (cameraMonitor.GetPose() != null) {
                    return;
                }
            }

            for (int i = 0; i < 20; i++) {
                baseAuto.drive(hex_motor_ticks * 2, hex_motor_ticks * 2, -hex_motor_ticks * 2, -hex_motor_ticks * 2, 0.5);
                if (cameraMonitor.GetPose() != null) {
                    return;
                }
            }
        }
    }

    private boolean AprilTagHoming() {
        // switch to power mode
        baseAuto.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseAuto.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseAuto.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseAuto.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double Y = cameraMonitor.GetY();
        double X = cameraMonitor.GetX();
        double Yaw = cameraMonitor.GetYaw();
        double power1 = 0;
        double power2 = 0;
        double power3 = 0;
        double seeNoPower = 0;
        double Xlimit = 5;
        double Yawlimit = 4;

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
        } else if (Y >= 12) {
            power3 = -0.3;
            isStrafe = true;
            setStatus = "Target";
        } else if (Y <= 11) {
            power3 = 0.3;
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
            baseAuto.FrontLeft.setPower(power1);
            baseAuto.BackLeft.setPower(power1);
            baseAuto.BackRight.setPower(power1);
            baseAuto.FrontRight.setPower(power1);
        }

        //power 2 = second set turn
        else if (isTurn) {
            baseAuto.FrontLeft.setPower(-power2);
            baseAuto.BackLeft.setPower(-power2);
            baseAuto.BackRight.setPower(power2);
            baseAuto.FrontRight.setPower(power2);
        }

        //power 3 = third set that strafe to target
        else if (isStrafe) {
            baseAuto.FrontLeft.setPower(power3);
            baseAuto.BackLeft.setPower(-power3);
            baseAuto.BackRight.setPower(power3);
            baseAuto.FrontRight.setPower(-power3);
        }

        // seeNoPower = nothing is see just goes in circles
        else {
            baseAuto.FrontLeft.setPower(seeNoPower);
            baseAuto.BackLeft.setPower(seeNoPower);
            baseAuto.BackRight.setPower(-seeNoPower);
            baseAuto.FrontRight.setPower(-seeNoPower);


        }
        //telemetry.addData(setStatus, "status");
        // telemetry.update();
        // try {
        // Thread.sleep(50);
        // } catch (InterruptedException e) {
        //   throw new RuntimeException(e);
        // }

        return !(Double.isNaN(Y) && Double.isNaN(X) && Double.isNaN(Yaw));
    }

    void StrafeInInches(int inchrs)
    {
        int strafeY = (int) (inchrs * 57.0);
        baseAuto.drive(strafeY * 1, -strafeY * 1, -strafeY * 1, strafeY * 1, 0.3);
    }

    void MoveFor(int inchrs)
    {
        int Forx = (int) (inchrs * 45.0);
        baseAuto.drive(Forx * 1, Forx * 1, Forx * 1, Forx * 1, 0.3);
    }

    void JumpToAprilTag() {
        double Y = cameraMonitor.GetY();
        Y = Y - 16;

        double X = cameraMonitor.GetCalculatedX();

        double Yaw = cameraMonitor.GetYaw();
        int hex_motor_ticks = 288;
        int turnYaw = (int) (Yaw * -13.5);
        int forX = (int) (X * 45.0);
        int strafeY = (int) (Y * 57.0);
        // rotate by yaw
        baseAuto.drive(-turnYaw * 1, -turnYaw * 1, turnYaw * 1, turnYaw * 1, 0.3);

        // move in X
        //drive(hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.3);
        baseAuto.drive(forX * 1, forX * 1, forX * 1, forX * 1, 0.3);
        // move in Y

        baseAuto.drive(strafeY * 1, -strafeY * 1, -strafeY * 1, strafeY * 1, 0.3);
    }

    public boolean GoToAprilTag() {
        AprilTagPoseFtc pose = cameraMonitor.GetPose();
        if (pose != null) {
            double range = pose.range;
            range = range - 16;

            double bearing = pose.bearing;
            int hex_motor_ticks = 288;
            int turn = (int) (bearing * -13.5);
            int strafe = (int) (range * -57.0);

            // rotate
            baseAuto.drive(-turn * 1, -turn * 1, turn * 1, turn * 1, 0.3);

            // strafe
            baseAuto.drive(-strafe * 1, strafe * 1, strafe * 1, -strafe * 1, 0.3);

            if (range < 1)
            {
                return true;
            }
        }

        return false;
    }

}



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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
        FtcOmniDrive,
        Homed,
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
        baseAuto.telemetry.addData("Frame rate", cameraMonitor.FrameRate);
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
                        state = HomingState.FtcOmniDrive;//HomingState.CenterOnTag;

                        // wait a bit to make sure tag refreshes
                        Sleep(500);
                    }
                } else if (state == HomingState.FtcOmniDrive) {
                    FtcOmniDrive();
                    if (IsOmniHomed())
                    {
                        state = HomingState.Homed;
                        break;
                    }
                }
                else if (state == HomingState.CenterOnTag) {
                    JumpToAprilTag();

                    if (Math.abs(cameraMonitor.GetYaw()) < 2) {
                        state = HomingState.FineTune;

                        // wait a bit to make sure tag refreshes
                        Sleep(500);
                    }
                } else if (state == HomingState.FineTune) {
                    /*
                    if (cameraMonitor.GetPose() == null)
                    {
                        state = HomingState.ScanForTag;
                        continue;
                    }
                    */

                    AprilTagHoming();

                    if (IsHomed()) {
                        state = HomingState.Homed;
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
            double x = pose.x;
            double yaw = Math.abs(pose.yaw);
            if (y < 13 && y > 11 && yaw < 3 && Math.abs(x) < 3) {
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
        double Xlimit = 2;
        double Yawlimit = 2;

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
            power1 = 0.2;
            isFor = true;
            setStatus = "Forward";
        } else if (X < -Xlimit) {
            // move forward
            power1 = -0.2;
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
            power2 = 0.2;
            isTurn = true;
            setStatus = "turn";
        } else if (Yaw <= -Yawlimit) {
            power2 = -0.2;
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
        //Sleep(5000);
        double Y = cameraMonitor.GetY();
        double X = cameraMonitor.GetCalculatedX();
        Y = Y - 16;
        X = X - 3;

        if (Math.abs(cameraMonitor.GetYaw()) > 45 && Y < 3)
        {
            //Y is wrong at extreme angle
            Y = -24;
        }



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

    final double DESIRED_DISTANCE = 14.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.25;//0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.10;//0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.10;//0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public void FtcOmniDrive()
    {

        AprilTagPoseFtc ftcPose = cameraMonitor.GetPose();

        if (ftcPose == null) return;

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  rangeError      = (ftcPose.range - DESIRED_DISTANCE);
        double  headingError    = ftcPose.bearing;
        double  yawError        = ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double strafe = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        //strafe
        double drive = - Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        /*
// Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        */

        double oomph = 0.06;

        if (drive > 0) drive += oomph;
        else drive -= oomph;

        if (strafe > 0) strafe += oomph;
        else strafe -= oomph;

        if (turn > 0) turn += oomph;
        else turn -= oomph;

        baseAuto.telemetry.addData("drive", drive);
        baseAuto.telemetry.addData("strafe", strafe);
        baseAuto.telemetry.addData("turn", turn);

        moveRobot(drive, strafe, turn);
    }

    public void moveRobot(double x, double y, double yaw) {
        // switch to power mode
        baseAuto.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseAuto.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseAuto.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseAuto.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        baseAuto.FrontLeft.setPower(leftFrontPower);
        baseAuto.FrontRight.setPower(rightFrontPower);
        baseAuto.BackLeft.setPower(leftBackPower);
        baseAuto.BackRight.setPower(rightBackPower);
    }

    public boolean IsOmniHomed() {
        AprilTagPoseFtc pose = cameraMonitor.GetPose();
        if (pose != null) {

            double range = pose.range;
            double yaw = pose.yaw;
            double bearing = Math.abs(pose.bearing);
            if (Math.abs(range - DESIRED_DISTANCE) < 0.25 && Math.abs(bearing) < 1 && Math.abs(yaw) < 1) {
                return true;
            }
        }
        return false;
    }


}



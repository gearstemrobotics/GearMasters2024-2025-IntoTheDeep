package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class AprilNaviOdo {
    private BaseOdoAuto baseOdoAuto;
    private CameraMonitor cameraMonitor;

    private double rangeGoalSet;

    private double yGoalSet = 12;

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

    public AprilNaviOdo(BaseOdoAuto Homing) {
        baseOdoAuto = Homing;
        cameraMonitor = new CameraMonitor(baseOdoAuto.webcamName);
        Thread t1 = new Thread(cameraMonitor, "t1");
        t1.start();

    }


    int LoopCount = 0;


    private void AddTelemtry() {
        baseOdoAuto.telemetry.addData("state", state.toString());
        baseOdoAuto.telemetry.addData("x(calc)", String.format("%.2f", cameraMonitor.GetCalculatedX()));
        baseOdoAuto.telemetry.addData("x", String.format("%.2f", cameraMonitor.GetX()));
        baseOdoAuto.telemetry.addData("y", String.format("%.2f", cameraMonitor.GetY()));
        baseOdoAuto.telemetry.addData("z", String.format("%.2f", cameraMonitor.GetZ()));
        baseOdoAuto.telemetry.addData("yaw", String.format("%.2f", cameraMonitor.GetYaw()));
        baseOdoAuto.telemetry.addData("range", String.format("%.2f", cameraMonitor.GetRange()));
        baseOdoAuto.telemetry.addData("bearing", String.format("%.2f", cameraMonitor.GetBearing()));
        baseOdoAuto.telemetry.addData("Frame rate", cameraMonitor.FrameRate);
        baseOdoAuto.telemetry.addData("LoopCount", LoopCount);
        baseOdoAuto.telemetry.update();
    }


    public void Home(double goal) {
        // yGoalSet = yGoal;
        //offset
        rangeGoalSet = goal + 3;
        DESIRED_DISTANCE = goal;
        // kickoff thread for camera here
        boolean hasJumped = false;
        state = HomingState.WaitForCamera;

        while (baseOdoAuto.opModeIsActive()) {
            LoopCount++;
            AddTelemtry();
            baseOdoAuto.telemetry.update();
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
                    if (cameraMonitor.GetRange() < rangeGoalSet || Math.abs(cameraMonitor.GetYaw()) > 55) {
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
                /*
                else if (state == HomingState.CenterOnTag) {
                    JumpToAprilTag();

                    if (Math.abs(cameraMonitor.GetYaw()) < 2) {
                        state = HomingState.FineTune;

                        // wait a bit to make sure tag refreshes
                        Sleep(500);
                    }
                } else if (state == HomingState.FineTune) {
                    //if (cameraMonitor.GetPose() == null)
                    //{
                    //    state = HomingState.ScanForTag;
                    //    continue;
                    //}

                    AprilTagHoming();

                    if (IsHomed()) {
                        state = HomingState.Homed;
                        // break out of the loop, we are done
                        break;
                    }
                }
                */
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
            if (y < yGoalSet + 1 && y > yGoalSet - 1 && yaw < 3 && Math.abs(x) < 3) {
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
                baseOdoAuto.drive(hex_motor_ticks * 2, hex_motor_ticks * 2, -hex_motor_ticks * 2, -hex_motor_ticks * 2, 0.1);
                if (cameraMonitor.GetPose() != null) {
                    return;
                }
            }

            for (int i = 0; i < 20; i++) {
                baseOdoAuto.drive(-hex_motor_ticks * 2, -hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.1);
                if (cameraMonitor.GetPose() != null) {
                    return;
                }
            }
        }
    }

    private boolean AprilTagHoming() {
        // switch to power mode
        baseOdoAuto.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseOdoAuto.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseOdoAuto.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseOdoAuto.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        } else if (Y >= yGoalSet) {
            power3 = -0.3;
            isStrafe = true;
            setStatus = "Target";
        } else if (Y <= yGoalSet -1) {
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
            baseOdoAuto.FrontLeft.setPower(power1);
            baseOdoAuto.BackLeft.setPower(power1);
            baseOdoAuto.BackRight.setPower(power1);
            baseOdoAuto.FrontRight.setPower(power1);
        }

        //power 2 = second set turn
        else if (isTurn) {
            baseOdoAuto.FrontLeft.setPower(-power2);
            baseOdoAuto.BackLeft.setPower(-power2);
            baseOdoAuto.BackRight.setPower(power2);
            baseOdoAuto.FrontRight.setPower(power2);
        }

        //power 3 = third set that strafe to target
        else if (isStrafe) {
            baseOdoAuto.FrontLeft.setPower(power3);
            baseOdoAuto.BackLeft.setPower(-power3);
            baseOdoAuto.BackRight.setPower(power3);
            baseOdoAuto.FrontRight.setPower(-power3);
        }

        // seeNoPower = nothing is see just goes in circles
        else {
            baseOdoAuto.FrontLeft.setPower(seeNoPower);
            baseOdoAuto.BackLeft.setPower(seeNoPower);
            baseOdoAuto.BackRight.setPower(-seeNoPower);
            baseOdoAuto.FrontRight.setPower(-seeNoPower);


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
        baseOdoAuto.drive(strafeY * 1, -strafeY * 1, -strafeY * 1, strafeY * 1, 0.3);
    }

    void MoveFor(int inchrs)
    {
        int Forx = (int) (inchrs * 45.0);
        baseOdoAuto.drive(Forx * 1, Forx * 1, Forx * 1, Forx * 1, 0.3);
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
        baseOdoAuto.drive(-turnYaw * 1, -turnYaw * 1, turnYaw * 1, turnYaw * 1, 0.3);

        // move in X
        //drive(hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.3);
        baseOdoAuto.drive(forX * 1, forX * 1, forX * 1, forX * 1, 0.3);
        // move in Y

        baseOdoAuto.drive(strafeY * 1, -strafeY * 1, -strafeY * 1, strafeY * 1, 0.3);
    }

    public boolean GoToAprilTag() {
        AprilTagPoseFtc pose = cameraMonitor.GetPose();
        if (pose != null) {
            double range = pose.range;
            range = range - rangeGoalSet;
            if (range > 50)
            {
                range = 50;
            }

            double bearing = pose.bearing;
            int hex_motor_ticks = 288;
            int turn = (int) (bearing * -13.5);
            int strafe = (int) (range *  57.0);

            // rotate
            baseOdoAuto.drive(-turn * 1, -turn * 1, turn * 1, turn * 1, 0.3);
            // drive
            baseOdoAuto.drive(strafe * 1, -strafe * 1, -strafe * 1, strafe * 1, 0.3);



            if (range < 1)
            {
                return true;
            }
        }

        return false;
    }

    double DESIRED_DISTANCE = 14.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.25;//0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.10;//0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.10;//0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final double HOMED_RANGE_TOLERANCE = 0.25;
    final double HOMED_BEARING_TOLERANCE = 0.5;
    final double HOMED_YAW_TOLERANCE = 0.5;

    // Oscillation fix variables
    private int stableHomingCount = 0;

    public void FtcOmniDrive()
    {

        AprilTagPoseFtc ftcPose = cameraMonitor.GetPose();

        if (ftcPose == null) return;

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  rangeError      = (ftcPose.range - DESIRED_DISTANCE);
        double  headingError    = ftcPose.bearing;
        double  yawError        = ftcPose.yaw;

        // OSCILLATION FIX 1: Add deadband to prevent micro-movements
        if (Math.abs(rangeError) < HOMED_RANGE_TOLERANCE) rangeError = 0;
        if (Math.abs(headingError) < HOMED_BEARING_TOLERANCE) headingError = 0;
        if (Math.abs(yawError) < HOMED_YAW_TOLERANCE) yawError = 0;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        //strafe
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        // OSCILLATION FIX 2: Replace "oomph" with smart minimum power
        final double MIN_POWER = 0.10; // Minimum power to overcome robot weight

        if (Math.abs(drive) > 0 && Math.abs(drive) < MIN_POWER) {
            drive = drive > 0 ? MIN_POWER : -MIN_POWER;
        }
        if (Math.abs(strafe) > 0 && Math.abs(strafe) < MIN_POWER) {
            strafe = strafe > 0 ? MIN_POWER : -MIN_POWER;
        }
        if (Math.abs(turn) > 0 && Math.abs(turn) < MIN_POWER) {
            turn = turn > 0 ? MIN_POWER : -MIN_POWER;
        }

        baseOdoAuto.telemetry.addData("drive", drive);
        baseOdoAuto.telemetry.addData("strafe", strafe);
        baseOdoAuto.telemetry.addData("turn", turn);

        moveRobot(-strafe, drive, turn);
    }

    public void moveRobot(double x, double y, double yaw) {
        // switch to power mode
        baseOdoAuto.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseOdoAuto.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseOdoAuto.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseOdoAuto.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        baseOdoAuto.FrontLeft.setPower(leftFrontPower);
        baseOdoAuto.FrontRight.setPower(rightFrontPower);
        baseOdoAuto.BackLeft.setPower(leftBackPower);
        baseOdoAuto.BackRight.setPower(rightBackPower);
    }

    // OSCILLATION FIX 3: Improved homing check with stability requirement
    public boolean IsOmniHomed() {
        AprilTagPoseFtc pose = cameraMonitor.GetPose();
        if (pose == null) {
            stableHomingCount = 0;
            return false;
        }

        double range = pose.range;
        double yaw = pose.yaw;
        double bearing = Math.abs(pose.bearing);

        // Check if within tolerance (slightly relaxed tolerances)
        if (Math.abs(range - DESIRED_DISTANCE) <= HOMED_RANGE_TOLERANCE && Math.abs(bearing) <= HOMED_BEARING_TOLERANCE && Math.abs(yaw) <= HOMED_YAW_TOLERANCE) {
            stableHomingCount++;
            return stableHomingCount >= 5; // Must be stable for 5 cycles to prevent oscillation
        } else {
            stableHomingCount = 0;
            return false;
        }
    }


}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@Autonomous
public class NoFormatJustTestOdo extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DcMotor BackLeft;

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;

    private void MoveOdo(double DesiredX, double DesiredY, double DesiredHeading) {
        odo.resetPosAndIMU();



        double DESIRED_DISTANCE = 14.0; //  this is how close the camera shou// ld get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
        final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.25;//0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE = 0.10;//0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN = 0.10;//0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double OdoX = odo.getPosition().getX(DistanceUnit.MM);
            double OdoY = odo.getPosition().getY(DistanceUnit.MM);
            double OdoHead = odo.getPosition().getHeading(AngleUnit.DEGREES);

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            double xError = DesiredX - OdoX;
            double yError = DesiredY - OdoY;
            double headingErrorOdo = DesiredHeading;
            //double DesiredRange =
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.

            // see if we need to go backwards
            double rangeSign = 1.0;
            if (Math.abs(xError) > Math.abs(yError) && xError < 0)
            {
                rangeSign = -1.0;
            }
            else if (Math.abs(yError) > Math.abs(xError) && yError < 0)
            {
                rangeSign = -1.0;
            }

            double rangeError = rangeSign * Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));//(ftcPose.range - DESIRED_DISTANCE);
            double headingError = Math.atan(yError / xError);//ftcPose.bearing;
            double yawError = -OdoHead + DesiredHeading; //ftcPose.yaw;
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("rangeError", rangeError);
            telemetry.addData("headingError", headingError);
            telemetry.addData("yawError", yawError);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            //strafe
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        /*
// Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        */

            double oomph = 0.0; // 0.06;

            if (drive > 0) drive += oomph;
            else drive -= oomph;

            if (strafe > 0) strafe += oomph;
            else strafe -= oomph;

            if (turn > 0) turn += oomph;
            else turn -= oomph;

            telemetry.addData("drive", drive);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.update();

            moveRobot(drive, strafe, turn);


        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // switch to power mode
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

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
        FrontLeft.setPower(leftFrontPower);
        FrontRight.setPower(rightFrontPower);
        BackLeft.setPower(leftBackPower);
        BackRight.setPower(rightBackPower);
    }

    @Override

    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        // FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        odo.setOffsets(-83.5, 12.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                odo.resetPosAndIMU();
                //simpleForward(10, 0);
                // break;

                MoveOdo(10, 0, 0);
                break;
            }
        }
    }
}

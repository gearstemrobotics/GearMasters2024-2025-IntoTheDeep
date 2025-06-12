package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Simple Navigation System using GoBilda Pinpoint odometry
 * Basic proportional control with working coordinate system
 */
public class SimpleNavigationSystem {

    // Hardware components
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private GoBildaPinpointDriver odo = null;

    // Simple tolerances
    private final double POSITION_TOLERANCE_INCHES = 0.25;
    private final double HEADING_TOLERANCE_DEGREES = 0.5;

    // Simple control gains - just proportional
    private final double DRIVE_P = 0.1;
    private final double STRAFE_P = 0.1;
    private final double TURN_P = 0.05;

    // Power limits
    private final double MAX_POWER = 0.6;
    private final double MIN_POWER = 0.15;

    private Telemetry telemetry;
    private int stableCycles = 0;

    /**
     * Initialize the navigation system
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");

        // Set motor directions - all reversed except front right
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors to run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //odo.setOffsets(-83.5, 12.5, DistanceUnit.MM);
        odo.setOffsets(-57.5, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Status", "Simple navigation initialized");
        telemetry.update();
    }

    /**
     * Simple navigation method
     */
    public boolean moveUntilPositioned(double targetX, double targetY, double targetHeading) {
        // Get current position (convert mm to inches)
        odo.update();
        Pose2D pos = odo.getPosition();
        double currentX = pos.getX(DistanceUnit.INCH);//getCurrentX();
        double currentY = pos.getY(DistanceUnit.INCH);//getCurrentY();
        double currentHeading = -odo.getHeading(UnnormalizedAngleUnit.DEGREES);//getCurrentHeading();

        // Calculate errors
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double headingError = normalizeAngle(targetHeading - currentHeading); // Fixed direction

        // Calculate distance to target
        double distanceToTarget = Math.sqrt(errorX * errorX + errorY * errorY);

        // Check if we've reached the target
        boolean atPosition = distanceToTarget < POSITION_TOLERANCE_INCHES;
        boolean atHeading = Math.abs(headingError) < HEADING_TOLERANCE_DEGREES;

        if (atPosition && atHeading) {
            stableCycles++;
            if (stableCycles >= 3) {
                setDrivePower(0, 0, 0);
                telemetry.addData("Status", "TARGET REACHED!");
                return true;
            }
        } else {
            stableCycles = 0;
        }

        // Simple proportional control
        double driveOutput = errorX * DRIVE_P;
        double strafeOutput = -errorY * STRAFE_P;  // Negated for correct direction
        double turnOutput = -headingError * TURN_P; // Negated for correct direction

        // Apply power limits
        driveOutput = limitPower(driveOutput);
        strafeOutput = limitPower(strafeOutput);
        turnOutput = limitPower(turnOutput);

        // Set drive powers
        setDrivePower(strafeOutput, driveOutput, -turnOutput);

        // Simple telemetry
        telemetry.addData("Target", "X=%.1f, Y=%.1f, H=%.1f", targetX, targetY, targetHeading);
        telemetry.addData("Current", "X=%.1f, Y=%.1f, H=%.1f", currentX, currentY, currentHeading);
        telemetry.addData("Error", "dX=%.2f, dY=%.2f, dH=%.1f", errorX, errorY, headingError);
        telemetry.addData("Distance", "%.2f inches", distanceToTarget);
        telemetry.addData("Powers", "D=%.2f, S=%.2f, T=%.2f", driveOutput, strafeOutput, turnOutput);
        telemetry.addData("Stable", "%d/3", stableCycles);

        return false;
    }

    /**
     * Limit power with minimum threshold
     */
    private double limitPower(double power) {
        if (Math.abs(power) < 0.02) return 0; // Deadband

        if (power > 0) {
            return Math.max(MIN_POWER, Math.min(MAX_POWER, power));
        } else if (power < 0) {
            return Math.max(-MAX_POWER, Math.min(-MIN_POWER, power));
        }
        return 0;
    }

    /**
     * Set drive motor powers - with coordinate fix
     */
    private void setDrivePower(double drive, double strafe, double turn) {
        // Mecanum drive with swapped drive/strafe
        double frontLeftPower = strafe + drive + turn;
        double frontRightPower = strafe - drive - turn;
        double backLeftPower = strafe - drive + turn;
        double backRightPower = strafe + drive - turn;

        // Normalize if needed
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        telemetry.addData("Power=", "FL=%.1f, FR=%.1f, BL=%.1f, BR=%.1f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);


        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

    }

    /**
     * Normalize angle to [-180, 180] range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Get current X position
     */
    public double getCurrentX() {
        odo.update();
        return odo.getPosX(DistanceUnit.INCH) / 25.4;
    }

    /**
     * Get current Y position
     */
    public double getCurrentY() {
        odo.update();
        return odo.getPosY(DistanceUnit.INCH) / 25.4;
    }

    /**
     * Get current heading
     */
    public double getCurrentHeading() {
        odo.update();
        return odo.getHeading(UnnormalizedAngleUnit.DEGREES);
    }

    /**
     * Reset position
     */
    public void resetPosition() {
        odo.resetPosAndIMU();
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        stableCycles = 0;
    }

    /**
     * Stop all motors
     */
    public void stop() {
        setDrivePower(0, 0, 0);
    }
}

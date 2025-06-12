package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * High-Precision Navigation System using GoBilda Pinpoint odometry
 * Features PID control, advanced filtering, and precision tuning
 */
public class NoWorkNavigationSystem {
    // PID constants for strafe (X movement) - separate tuning
    private static final double STRAFE_KP = 0.15;//0.08;   // Proportional
    private static final double STRAFE_KI = 0.004; // Integral
    private static final double STRAFE_KD = 0.006;  // Derivativepackage org.firstinspires.ftc.teamcode;

    // Hardware components
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private GoBildaPinpointDriver odo = null;

    // Precision tolerances - much tighter for high accuracy
    private static final double POSITION_TOLERANCE_INCHES = 0.25;  // Quarter inch precision
    private static final double HEADING_TOLERANCE_DEGREES = 1.0;   // 1 degree precision
    private static final double FINE_POSITION_TOLERANCE = 0.2;     // Very fine final positioning
    private static final double FINE_HEADING_TOLERANCE = 1;      // Very fine heading

    // Speed control parameters
    private static final double MAX_DRIVE_SPEED = 0.6;       // Reduced for precision
    private static final double MIN_DRIVE_SPEED = 0.3;      // Very slow for final approach
    private static final double MAX_TURN_SPEED = 0.3;        // Reduced turn speed
    private static final double MIN_TURN_SPEED = 0.1;       // Very slow turning

    // PID constants for drive (X/Y movement)
    private static final double DRIVE_KP = 0.15;//0.08;    // Proportional
    private static final double DRIVE_KI = 0.004;  // Integral
    private static final double DRIVE_KD = 0.006;   // Derivative

    // PID constants for turning (heading)
    private static final double TURN_KP = 0.02;    // Proportional
    private static final double TURN_KI = 0.0005;   // Integral
    private static final double TURN_KD = 0.002;    // Derivative

    // Advanced control parameters
    private static final double INTEGRAL_LIMIT = 0.25;       // Prevent integral windup
    private static final double DERIVATIVE_FILTER = 0.8;     // Filter derivative noise
    private static final double DEADBAND_INCHES = 0.03;     // Ignore tiny movements
    private static final double HEADING_DEADBAND = 0.8;     // Ignore tiny rotations

    // Ramping and settling
    private static final double ACCELERATION_LIMIT = 0.025;  // Max power change per cycle
    private static final double SETTLING_TIME_MS = 150;      // Time to settle at target
    private static final int STABLE_CYCLES_NEEDED = 5;       // Cycles within tolerance

    // PID state variables for drive
    private double driveIntegralX = 0, driveIntegralY = 0;
    private double lastErrorX = 0, lastErrorY = 0;
    private double filteredDerivX = 0, filteredDerivY = 0;

    // PID state variables for turn
    private double turnIntegral = 0;
    private double lastHeadingError = 0;
    private double filteredTurnDeriv = 0;

    // Power ramping
    private double lastDrivePower = 0, lastStrafePower = 0, lastTurnPower = 0;

    // Settling detection
    private ElapsedTime settlingTimer = new ElapsedTime();
    private int stableCycles = 0;
    private boolean inFinalApproach = false;

    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime cycleTimer = new ElapsedTime();

    /**
     * Initialize the navigation system with high precision settings
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

        // Set motors to brake for maximum precision
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and configure encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize GoBilda Pinpoint odometry with precision settings
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set precise odometry offsets in mm
        odo.setOffsets(-83.5, 12.5); // X offset: -83.5mm, Y offset: 12.5mm

        // Use high-resolution pod setting
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions - both reversed
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Recalibrate IMU for best precision
        odo.recalibrateIMU();

        // Reset position
        odo.resetPosAndIMU();

        // Initialize timers
        cycleTimer.reset();
        settlingTimer.reset();

        telemetry.addData("Status", "High-precision navigation initialized");
        telemetry.update();
    }

    /**
     * High-precision navigation method with PID control and settling detection
     */
    public boolean moveUntilPositioned(double targetX, double targetY, double targetHeading) {
        // Update odometry
        odo.update();

        // Get current position (convert mm to inches)
        double currentX = odo.getPosX() / 25.4;
        double currentY = odo.getPosY() / 25.4;
        double currentHeading = Math.toDegrees(odo.getHeading());

        // Calculate errors
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double headingError = -normalizeAngle(targetHeading - currentHeading);

        // Calculate distance to target
        double distanceToTarget = Math.sqrt(errorX * errorX + errorY * errorY);

        // Determine if we're in final approach mode
        boolean wasInFinalApproach = inFinalApproach;

        inFinalApproach = distanceToTarget < 3.0 && Math.abs(headingError) < 10; // Within 3 inches and 10 degrees

        // Smooth transition to final approach - reduce integral windup but don't reset
        if (inFinalApproach && !wasInFinalApproach) {
            // Scale down integrals instead of resetting completely
            driveIntegralX *= 0.3;
            driveIntegralY *= 0.3;
            turnIntegral *= 0.3;
            telemetry.addData("Transition", "Entering final approach mode");
        }

        // Choose tolerances based on approach phase
        double positionTol = inFinalApproach ? FINE_POSITION_TOLERANCE : POSITION_TOLERANCE_INCHES;
        double headingTol = inFinalApproach ? FINE_HEADING_TOLERANCE : HEADING_TOLERANCE_DEGREES;

        // Check if we're within tolerance
        boolean atPosition = distanceToTarget < positionTol;
        boolean atHeading = Math.abs(headingError) < headingTol;

        if (atPosition && atHeading) {
            stableCycles++;
            if (stableCycles >= STABLE_CYCLES_NEEDED) {
                // We've been stable long enough - target reached!
                setDrivePower(0, 0, 0);
                telemetry.addData("Status", "TARGET REACHED - High Precision!");
                return true;
            }
        } else {
            stableCycles = 0;
        }

        // Apply deadband to prevent oscillation
        if (Math.abs(errorX) < DEADBAND_INCHES) errorX = 0;
        if (Math.abs(errorY) < DEADBAND_INCHES) errorY = 0;
        if (Math.abs(headingError) < HEADING_DEADBAND) headingError = 0;

        // Calculate cycle time for derivatives
        double deltaTime = cycleTimer.seconds();
        cycleTimer.reset();
        if (deltaTime == 0) deltaTime = 0.02; // Prevent division by zero

        // PID control for X and Y
        driveIntegralY += errorY * deltaTime;
        driveIntegralY = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, driveIntegralY));
        double driveDerivative = (errorY - lastErrorY) / deltaTime;
        filteredDerivY = filteredDerivY * DERIVATIVE_FILTER + driveDerivative * (1 - DERIVATIVE_FILTER);
        double driveOutput = errorY * DRIVE_KP + driveIntegralY * DRIVE_KI + filteredDerivY * DRIVE_KD;

        driveIntegralX += errorX * deltaTime;
        driveIntegralX = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, driveIntegralX));
        double strafeDerivative = (errorX - lastErrorX) / deltaTime;
        filteredDerivX = filteredDerivX * DERIVATIVE_FILTER + strafeDerivative * (1 - DERIVATIVE_FILTER);
        double strafeOutput = errorX * STRAFE_KP + driveIntegralX * STRAFE_KI + filteredDerivX * STRAFE_KD;

        // PID control for heading
        turnIntegral += headingError * deltaTime;
        turnIntegral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, turnIntegral));
        double turnDerivative = (headingError - lastHeadingError) / deltaTime;
        filteredTurnDeriv = filteredTurnDeriv * DERIVATIVE_FILTER + turnDerivative * (1 - DERIVATIVE_FILTER);
        double turnOutput = headingError * TURN_KP + turnIntegral * TURN_KI + filteredTurnDeriv * TURN_KD;

        // Update PID state
        lastErrorX = errorX;
        lastErrorY = errorY;
        lastHeadingError = headingError;

        // Apply speed limits based on distance
        double speedScale = calculatePrecisionSpeedScale(distanceToTarget);
        driveOutput *= speedScale;
        strafeOutput *= speedScale;

        // Limit turn output
        double turnScale = calculateTurnSpeedScale(Math.abs(headingError));
        turnOutput *= turnScale;

        // Apply acceleration limiting for smooth motion
        driveOutput = applyAccelerationLimit(driveOutput, lastDrivePower);
        strafeOutput = applyAccelerationLimit(strafeOutput, lastStrafePower);
        turnOutput = applyAccelerationLimit(turnOutput, lastTurnPower);

        // Store for next cycle
        lastDrivePower = driveOutput;
        lastStrafePower = strafeOutput;
        lastTurnPower = turnOutput;

        // Apply drive commands
        setDrivePower(strafeOutput, -driveOutput,  turnOutput);

        // Enhanced telemetry with PID debugging
        telemetry.addData("Mode", inFinalApproach ? "FINAL APPROACH" : "APPROACHING");
        telemetry.addData("Target", "X=%.2f, Y=%.2f, H=%.1f", targetX, targetY, targetHeading);
        telemetry.addData("Current", "X=%.2f, Y=%.2f, H=%.1f", currentX, currentY, currentHeading);
        telemetry.addData("Error", "dX=%.3f, dY=%.3f, dH=%.2f", errorX, errorY, headingError);
        telemetry.addData("Distance", "%.3f inches", distanceToTarget);
        telemetry.addData("Outputs", "D=%.3f, S=%.3f, T=%.3f", driveOutput, strafeOutput, turnOutput);
        telemetry.addData("Integrals", "X=%.3f, Y=%.3f, H=%.3f", driveIntegralX, driveIntegralY, turnIntegral);
        telemetry.addData("Stable Cycles", "%d/%d", stableCycles, STABLE_CYCLES_NEEDED);
        telemetry.addData("Speed Scale", "%.3f", speedScale);

        return false; // Still moving
    }

    /**
     * Calculate PID output with integral windup protection and derivative filtering
     */
    private double calculatePIDOutput(double error, double lastError, double integral,
                                      double filteredDeriv, double deltaTime,
                                      double kP, double kI, double kD) {
        // Proportional term
        double proportional = error * kP;

        // Integral term with windup protection
        integral += error * deltaTime;
        integral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integral));
        double integralTerm = integral * kI;

        // Derivative term with filtering
        double derivative = (error - lastError) / deltaTime;
        filteredDeriv = filteredDeriv * DERIVATIVE_FILTER + derivative * (1 - DERIVATIVE_FILTER);
        double derivativeTerm = filteredDeriv * kD;

        // Update the integral values (this was missing before!)
        if (error == lastError) { // This is driveIntegralX
            driveIntegralX = integral;
        } else if (Math.abs(error) > Math.abs(lastError)) { // This is driveIntegralY
            driveIntegralY = integral;
        } else { // This is turnIntegral
            turnIntegral = integral;
        }

        return proportional + integralTerm + derivativeTerm;
    }

    /**
     * Calculate precision speed scaling
     */
    private double calculatePrecisionSpeedScale(double distance) {
        if (distance > 12.0) {
            return MAX_DRIVE_SPEED;
        } else if (distance < 0.5) {
            return MIN_DRIVE_SPEED;
        } else {
            // Smooth exponential scaling for precision
            double normalizedDistance = distance / 12.0;
            double scale = MIN_DRIVE_SPEED + ((MAX_DRIVE_SPEED - MIN_DRIVE_SPEED) *
                    Math.pow(normalizedDistance, 0.7)); // Exponential curve
            return scale;
        }
    }

    /**
     * Calculate turn speed scaling
     */
    private double calculateTurnSpeedScale(double headingError) {
        if (headingError > 30.0) {
            return MAX_TURN_SPEED;
        } else if (headingError < 1.0) {
            return MIN_TURN_SPEED;
        } else {
            double normalizedError = headingError / 30.0;
            return MIN_TURN_SPEED + (MAX_TURN_SPEED - MIN_TURN_SPEED) *
                    Math.pow(normalizedError, 0.6);
        }
    }

    /**
     * Apply acceleration limiting for smooth motion
     */
    private double applyAccelerationLimit(double targetPower, double lastPower) {
        double powerChange = targetPower - lastPower;
        double maxChange = ACCELERATION_LIMIT;

        if (Math.abs(powerChange) > maxChange) {
            if (powerChange > 0) {
                return lastPower + maxChange;
            } else {
                return lastPower - maxChange;
            }
        }
        return targetPower;
    }

    /**
     * Reset PID state variables
     */
    private void resetPIDState() {
        driveIntegralX = 0;
        driveIntegralY = 0;
        turnIntegral = 0;
        lastErrorX = 0;
        lastErrorY = 0;
        lastHeadingError = 0;
        filteredDerivX = 0;
        filteredDerivY = 0;
        filteredTurnDeriv = 0;
        stableCycles = 0;
    }

    /**
     * Set drive motor powers with precision
     */
    private void setDrivePower(double drive, double strafe, double turn) {
        // Calculate wheel powers for mecanum drive
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        // Find the maximum absolute power
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        // Normalize if necessary, but preserve precision for small movements
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
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
     * Get current X position in inches with high precision
     */
    public double getCurrentX() {
        odo.update();
        return odo.getPosX() / 25.4;
    }

    /**
     * Get current Y position in inches with high precision
     */
    public double getCurrentY() {
        odo.update();
        return odo.getPosY() / 25.4;
    }

    /**
     * Get current heading in degrees with high precision
     */
    public double getCurrentHeading() {
        odo.update();
        return Math.toDegrees(odo.getHeading());
    }

    /**
     * Reset position and all PID state
     */
    public void resetPosition() {
        odo.resetPosAndIMU();
        resetPIDState();
        lastDrivePower = 0;
        lastStrafePower = 0;
        lastTurnPower = 0;
        inFinalApproach = false;
    }

    /**
     * Stop all drive motors immediately
     */
    public void stop() {
        setDrivePower(0, 0, 0);
        resetPIDState();
    }

    /**
     * Emergency stop with braking
     */
    public void emergencyStop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        resetPIDState();
    }
}

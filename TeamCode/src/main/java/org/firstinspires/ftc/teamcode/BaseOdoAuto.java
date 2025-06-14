package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public abstract class BaseOdoAuto extends LinearOpMode {

    protected SimpleNavigationSystem navigation = new SimpleNavigationSystem();

    protected TouchSensor TouchSen;
    // protected RevBlinkinLedDriver blinkinLedDriver;
    protected ColorSensor color;

    //protected DcMotor DumpArm;

    //protected DcMotor extendArmSideways;

    protected GoBildaPinpointDriver odo;

    protected WebcamName webcamName;
    protected DcMotor BackLeft;
    protected DcMotor FrontRight;
    protected DcMotor FrontLeft;
    protected DcMotor BackRight;
    //protected DcMotor extendArmUp;
    //protected DcMotor extendArmSideways;
    //protected DcMotor DumpArm;
    //protected CRServo gripper2;
    //protected CRServo gripper;

    double oldTime = 0;
    int anglePos;
    int liftPos;
    int extendPos;
    int FrontRightPos;
    int LeftArmPos;
    int BackRightPos;
    int RightArmPos;
    int FrontLeftPos;
    int BackLeftPos;

    protected EncoderMacrosForOdoAuto EncoderMacrosForOdoAutoTask;


    // int gripper2Pos;


    BaseOdoAuto() {
        extendPos = 0;
        liftPos = 0;
        anglePos = 0;
        FrontRightPos = 0;
        BackRightPos = 0;
        FrontLeftPos = 0;
        BackLeftPos = 0;


        // gripper2Pos = 0;

    }
    public void ThreadMaker()
    {
        EncoderMacrosForOdoAutoTask = new EncoderMacrosForOdoAuto(
                hardwareMap.get(DcMotor.class, "extendArmSideways"),
                hardwareMap.get(DcMotor.class, "extendArmUp"),
                hardwareMap.get(Servo.class, "OrientServo"),
                hardwareMap.get(Servo.class, "LevelServo"),
                hardwareMap.get(CRServo.class, "gripper"),
                hardwareMap.get(CRServo.class, "gripper2"),
                hardwareMap.get(ColorRangeSensor.class, "Color"),
                hardwareMap.get(DcMotor.class, "DumpArm"),
                hardwareMap.get(TouchSensor.class, "touch"),
                hardwareMap.get(DcMotor.class, "climbArm"));

        Thread t2 = new Thread(EncoderMacrosForOdoAutoTask, "t2");
        t2.start();
    }

    public void Home()
    {
        AprilNaviOdo aprilNaviOdo = new AprilNaviOdo(this);
        aprilNaviOdo.Home(14);
    }



    public void move(double x, double y, double heading)
    {
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !navigation.moveUntilPositioned(x, y, heading)) {
            telemetry.update();
            sleep(10); // Faster update for precision
        }
        sleep(1000); // Longer pause to observe precision
    }



/*


    //arm encoder stuff
    protected void arm(double ExtendArmTarget, double ExtendArmSidewaysTarget, double DumpArmTarget, double Speed) {
        extendPos += ExtendArmTarget;
        liftPos += ExtendArmSidewaysTarget;
        anglePos += DumpArmTarget;
        extendArmUp.setTargetPosition(extendPos);
        extendArmSideways.setTargetPosition(liftPos);
        DumpArm.setTargetPosition(anglePos);
        extendArmUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DumpArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmUp.setPower(Speed);
        extendArmSideways.setPower(Speed);
        DumpArm.setPower(Speed);
        while (opModeIsActive() && (extendArmUp.isBusy() || extendArmSideways.isBusy() || DumpArm.isBusy())) {
            // Do nothing
        }
        extendArmUp.setPower(0);
        extendArmSideways.setPower(0);
        DumpArm.setPower(0);
    }
    /*

    protected void everything( double FrontRightTarget, double BackRightTarget,
                               double FrontLeftTarget, double BackLeftTarget,
                               double ExtendArmTarget, double LiftArmTarget,
                               double AngleArmTarget, double ArmSpeed, double WheelSpeed) {
        extendPos += ExtendArmTarget;
        liftPos += LiftArmTarget;
        anglePos += AngleArmTarget;
        extendArmUp.setTargetPosition(extendPos);
        extendArmSideways.setTargetPosition(liftPos);
        DumpArm.setTargetPosition(anglePos);
        extendArmUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DumpArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmUp.setPower(ArmSpeed);
        extendArmSideways.setPower(ArmSpeed);
        DumpArm.setPower(ArmSpeed);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRightPos += FrontRightTarget;
        BackRightPos += BackRightTarget;
        FrontLeftPos += FrontLeftTarget;
        BackLeftPos += BackLeftTarget;
        FrontRight.setTargetPosition(FrontRightPos);
        BackRight.setTargetPosition(BackRightPos);
        FrontLeft.setTargetPosition(FrontLeftPos);
        BackLeft.setTargetPosition(BackLeftPos);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setPower(WheelSpeed);
        BackRight.setPower(WheelSpeed);
        FrontLeft.setPower(WheelSpeed);
        BackLeft.setPower(WheelSpeed);
        while (opModeIsActive() &&
                // (FrontRight.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy()) // while everything is busy
                (extendArmUp.isBusy() || extendArmSideways.isBusy() || DumpArm.isBusy() || FrontRight.isBusy() || BackRight.isBusy() || FrontLeft.isBusy() || BackLeft.isBusy()) // while anything is busy
        ) {
        }
        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        extendArmUp.setPower(0);
        extendArmSideways.setPower(0);
        DumpArm.setPower(0);
    }
     */


    protected void drive(double FrontRightTarget, double BackRightTarget,
                         double FrontLeftTarget, double BackLeftTarget, double Speed) {

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRightPos += FrontRightTarget;
        BackRightPos += BackRightTarget;
        FrontLeftPos += FrontLeftTarget;
        BackLeftPos += BackLeftTarget;
        FrontRight.setTargetPosition(FrontRightPos);
        BackRight.setTargetPosition(BackRightPos);
        FrontLeft.setTargetPosition(FrontLeftPos);
        BackLeft.setTargetPosition(BackLeftPos);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setPower(Speed);
        BackRight.setPower(Speed);
        FrontLeft.setPower(Speed);
        BackLeft.setPower(Speed);
        while (opModeIsActive() && (FrontRight.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy()))
            // while everything is busy
            // (FrontRight.isBusy() || BackRight.isBusy() || FrontLeft.isBusy() || BackLeft.isBusy()) // while anything is busy
         {
            // Do nothing
        }
        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
    }

    // runs after map and before wait for Start
    protected void RunInit()
    {
        //navigation.init(hardwareMap, telemetry);
    }

    protected abstract void RunOpModeInnerLoop();

    @Override
    public void runOpMode() {

        Map();

        RunInit();
        navigation.init(hardwareMap, telemetry);
        ThreadMaker();

        waitForStart();
        if (opModeIsActive()) {
            PrepMotor();
            RunOpModeInnerLoop();
        }
        EncoderMacrosForOdoAutoTask.stop();
    }

    protected void Map() {
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
       // extendArmUp = hardwareMap.get(DcMotor.class, "extendArmUp");
      //  extendArmSideways = hardwareMap.get(DcMotor.class, "extendArmSideways");
       // DumpArm = hardwareMap.get(DcMotor.class, "DumpArm");
       // gripper2 = hardwareMap.get(CRServo.class, "gripper2");
       // gripper = hardwareMap.get(CRServo.class, "gripper");
        //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
    }

    protected void PrepMotor() {


        // Set motor directions - all reversed except front right
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        //extendArmSideways.setDirection(DcMotorSimple.Direction.REVERSE);
        //extendArmUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //extendArmSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DumpArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // extendArmUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // extendArmSideways.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // DumpArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/*
        odo.setOffsets(-55.0, 50.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        Pose2D pos = odo.getPosition();

 */
    }
}


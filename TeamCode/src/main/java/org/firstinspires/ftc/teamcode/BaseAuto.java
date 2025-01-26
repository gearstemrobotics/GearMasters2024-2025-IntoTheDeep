package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class BaseAuto extends LinearOpMode {

    protected TouchSensor TouchSen;
    // protected RevBlinkinLedDriver blinkinLedDriver;
    protected ColorSensor color;

    protected CameraMonitor cameraMonitor;

    protected DcMotor BackLeft;
    protected DcMotor FrontRight;
    protected DcMotor FrontLeft;
    protected DcMotor BackRight;
    protected DcMotor extendArm;
    protected DcMotor liftArm;
    protected DcMotor angleArm;
    protected CRServo gripper2;


    int anglePos;
    int liftPos;
    int extendPos;
    int FrontRightPos;
    int LeftArmPos;
    int BackRightPos;
    int RightArmPos;
    int FrontLeftPos;
    int BackLeftPos;


    // int gripper2Pos;


    BaseAuto() {
        extendPos = 0;
        liftPos = 0;
        anglePos = 0;
        FrontRightPos = 0;
        BackRightPos = 0;
        FrontLeftPos = 0;
        BackLeftPos = 0;


        // gripper2Pos = 0;

    }

    //arm encoder stuff
    protected void arm(double ExtendArmTarget, double LiftArmTarget, double AngleArmTarget, double Speed) {
        extendPos += ExtendArmTarget;
        liftPos += LiftArmTarget;
        anglePos += AngleArmTarget;
        extendArm.setTargetPosition(extendPos);
        liftArm.setTargetPosition(liftPos);
        angleArm.setTargetPosition(anglePos);
        extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm.setPower(Speed);
        liftArm.setPower(Speed);
        angleArm.setPower(Speed);
        while (opModeIsActive() && (extendArm.isBusy() || liftArm.isBusy() || angleArm.isBusy())) {
            // Do nothing
        }
        extendArm.setPower(0);
        liftArm.setPower(0);
        angleArm.setPower(0);
    }

    protected void everything(double ExtendArmTarget, double LiftArmTarget, double AngleArmTarget, double FrontRightTarget, double BackRightTarget,
                              double FrontLeftTarget, double BackLeftTarget, double Speed) {
        extendPos += ExtendArmTarget;
        liftPos += LiftArmTarget;
        anglePos += AngleArmTarget;
        extendArm.setTargetPosition(extendPos);
        liftArm.setTargetPosition(liftPos);
        angleArm.setTargetPosition(anglePos);
        extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm.setPower(Speed);
        liftArm.setPower(Speed);
        angleArm.setPower(Speed);
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
        FrontRight.setPower(Speed);
        BackRight.setPower(Speed);
        FrontLeft.setPower(Speed);
        BackLeft.setPower(Speed);
        while (opModeIsActive() &&
                // (FrontRight.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy()) // while everything is busy
                (extendArm.isBusy() || liftArm.isBusy() || angleArm.isBusy() || FrontRight.isBusy() || BackRight.isBusy() || FrontLeft.isBusy() || BackLeft.isBusy()) // while anything is busy
        ) {
        }
        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        extendArm.setPower(0);
        liftArm.setPower(0);
        angleArm.setPower(0);
    }

    protected void drive(double FrontRightTarget, double BackRightTarget,
                         double FrontLeftTarget, double BackLeftTarget, double Speed) {

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
        FrontRight.setPower(Speed);
        BackRight.setPower(Speed);
        FrontLeft.setPower(Speed);
        BackLeft.setPower(Speed);
        while (opModeIsActive() &&
                (FrontRight.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy()) // while everything is busy
            // (FrontRight.isBusy() || BackRight.isBusy() || FrontLeft.isBusy() || BackLeft.isBusy()) // while anything is busy
        ) {
            // Do nothing
        }
        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
    }

    protected abstract void RunOpModeInnerLoop();

    @Override
    public void runOpMode() {

        Map();

        waitForStart();
        if (opModeIsActive()) {
            PrepMotor();
            RunOpModeInnerLoop();
        }
    }

    protected void Map() {
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        extendArm = hardwareMap.get(DcMotor.class, "extendArm");
        liftArm = hardwareMap.get(DcMotor.class, "liftArm");
        angleArm = hardwareMap.get(DcMotor.class, "angleArm");
        gripper2 = hardwareMap.get(CRServo.class, "gripper2");
        //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    protected void PrepMotor() {

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        liftArm.setDirection(DcMotorSimple.Direction.REVERSE);
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}


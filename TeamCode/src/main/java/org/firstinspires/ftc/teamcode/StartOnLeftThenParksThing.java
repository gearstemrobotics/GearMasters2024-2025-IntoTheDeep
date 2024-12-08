
        package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CameraMonitor;

@Autonomous
public class StartOnLeftThenParksThing extends LinearOpMode {

    private TouchSensor TouchSen;
    RevBlinkinLedDriver blinkinLedDriver;
    private ColorSensor color;

    private CameraMonitor cameraMonitor;

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor extendArm;
    private DcMotor liftArm;
    private DcMotor angleArm;


    int anglePos;
    int liftPos;
    int extendPos;
    int FrontRightPos;
    int LeftArmPos;
    int BackRightPos;
    int RightArmPos;
    int FrontLeftPos;
    int BackLeftPos;


    //arm encoder stuff
    private void arm(double ExtendArmTarget, double LiftArmTarget, double AngleArmTarget, double Speed) {
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

    private void drive(double FrontRightTarget, double BackRightTarget,
                       double FrontLeftTarget, double BackLeftTarget, double Speed) {

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
        while (opModeIsActive() && (FrontRight.isBusy() || BackRight.isBusy() || FrontLeft.isBusy() || BackLeft.isBusy())) {
            // Do nothing
        }
        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
    }


    @Override
    public void runOpMode() {


        int hex_motor_ticks;

        // WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        // cameraMonitor = new CameraMonitor(webcamName);
        // Thread t1 = new Thread(cameraMonitor, "t1");
        // t1.start();


        float vertical = 1;
        float horizontal = 0;
        float pivot = 0;


        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        extendArm = hardwareMap.get(DcMotor.class, "extendArm");
        liftArm = hardwareMap.get(DcMotor.class, "liftArm");
        angleArm = hardwareMap.get(DcMotor.class, "angleArm");
        //  Arm = hardwareMap.get(DcMotor.class, "Arm");
        // Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        // Arm3 = hardwareMap.get(DcMotor.class, "Arm3");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


        waitForStart();
        if (opModeIsActive()) {


            BackRight.setDirection(DcMotor.Direction.REVERSE);
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
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


            double mult = 4;

            while (opModeIsActive()) {


                sleep(500);
                hex_motor_ticks = 288;
                extendPos = 0;
                liftPos = 0;
                anglePos = 0;
                FrontRightPos = 0;
                BackRightPos = 0;
                FrontLeftPos = 0;
                BackLeftPos = 0;
                sleep(500);
                //############################################
                //WARNING MAY NOT WORK CTRL Z IT OR REVERSE THE ORDER
                //############################################
                //Lift arm and strafe left at the same time
                arm(0, hex_motor_ticks * 12,  0, 1 );
                sleep(1000);
                drive(-hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, 0.5);
                // sleep(1000);
                drive( -hex_motor_ticks * 8, hex_motor_ticks * 8, -hex_motor_ticks * 8, hex_motor_ticks * 8, 0.5);
                // sleep(1000);
                // moves the arm down
                arm(0, -hex_motor_ticks * 12,  0, 0.5 );
                // Moves to the wall
                sleep(3000);
                //drive(hex_motor_ticks * 8, -hex_motor_ticks * 8, hex_motor_ticks * 8, -hex_motor_ticks * 8, 0.5);
                //Moves to the corner
                drive( hex_motor_ticks * 7, -hex_motor_ticks * 7, hex_motor_ticks * 7, - hex_motor_ticks * 7  , 0.5);
                drive(-hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, -hex_motor_ticks * 7.5, 0.5);




                break;


            }
            //telemetry.addData("April Tags", cameraMonitor.GetIdsFound());
            //telemetry.addData("BackLeft.getCurrentPosition", BackLeft.getCurrentPosition());
            // telemetry.update();


        }
    }
}


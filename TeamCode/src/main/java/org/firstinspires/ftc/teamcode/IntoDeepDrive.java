package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp
public class IntoDeepDrive extends LinearOpMode {


    //RevBlinkinLedDriver blinkinLedDriver;
    private CRServo gripper;
    private CRServo gripper2;

    // private TouchSensor TouchSen;

    // private TouchSensor magSen;
    private ColorSensor color;

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;

    private DcMotor BackRight;
    private DcMotor liftArm;
    private DcMotor angleArm;
    private DcMotor extendArm;


    private DcMotor climbArm;
    private boolean Moving = false;

    private boolean useExpanasionHub = true;


    /*
    private int anglePos = 0;
    private int liftPos = 0;
    private int extendPos = 0;
    private int FrontRightPos = 0;
    private int LeftArmPos = 0;
    private int BackRightPos = 0;
    private int RightArmPos = 0;
    private int FrontLeftPos = 0;
    private int BackLeftPos = 0;


     */

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //DoWork();

        DoWork3();
    }


    public void DoWork3() {
        BackGroundMech task = new BackGroundMech(gamepad1,
                hardwareMap.get(DcMotor.class, "FrontRight"),
                hardwareMap.get(DcMotor.class, "FrontLeft"),
                hardwareMap.get(DcMotor.class, "BackRight"),
                hardwareMap.get(DcMotor.class, "BackLeft"));

        Thread t1 = new Thread(task, "t1");

        TestBackGroundEncodeMacro task2 = new TestBackGroundEncodeMacro(gamepad2, gamepad1,
                hardwareMap.get(DcMotor.class, "liftArm"),
                hardwareMap.get(DcMotor.class, "angleArm"),
                hardwareMap.get(DcMotor.class, "extendArm"),
                Moving);

        Thread t2 = new Thread(task2, "t2");

        //  blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");

        float power2 = 0;
        float power = 0;
        float power3 = 0;


        extendArm = hardwareMap.get(DcMotor.class, "extendArm");
        liftArm = hardwareMap.get(DcMotor.class, "liftArm");
        angleArm = hardwareMap.get(DcMotor.class, "angleArm");
        climbArm = hardwareMap.get(DcMotor.class, "climbArm");

        // TouchSen = hardwareMap.get(TouchSensor.class, "TouchSen");
        // magSen = hardwareMap.get(TouchSensor.class, "magSen");
        color = hardwareMap.get(ColorSensor.class, "Color");
        gripper = hardwareMap.get(CRServo.class, "gripper");
        gripper2 = hardwareMap.get(CRServo.class, "gripper2");
        color = hardwareMap.get(ColorSensor.class, "Color");
        gripper.resetDeviceConfigurationForOpMode();
        gripper2.resetDeviceConfigurationForOpMode();


        waitForStart();
        //t1.start();
        if (opModeIsActive()) {
            t1.start();
            t2.start();
            while (opModeIsActive()) {



                if (gamepad2.left_trigger > 0) {
                    gripper2.setPower(-1);
                    gripper.setPower(1);
                } else if (gamepad2.right_trigger > 0) {
                    gripper2.setPower(1);
                    gripper.setPower(-1);
                } else {
                    gripper2.setPower(0);
                    gripper.setPower(0);
                }


                telemetry.addData("power", power);
                telemetry.addData("power2", power2);
                telemetry.addData("power3", power3);
                // telemetry.addData("Touched", TouchSen.getValue());
                // telemetry.addData("Touched Magnet", magSen.getValue());
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();


            }
        }

    }
}
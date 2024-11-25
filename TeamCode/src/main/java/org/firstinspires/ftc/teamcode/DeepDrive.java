package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp(name = "DeepDrive")
public class DeepDrive extends LinearOpMode {


    RevBlinkinLedDriver blinkinLedDriver;
    private Servo gripper;
    private CRServo gripper2;

    private TouchSensor TouchSen;
    private ColorSensor color;

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;

    private DcMotor BackRight;

    private DcMotor extendArm;
    private DcMotor liftArm;
    private DcMotor angleArm;

    private boolean useExpanasionHub = false;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //DoWork();
        DoWork2();
    }

    public void DoWork2() {
        BackGroundMech task = new BackGroundMech(gamepad1,
                hardwareMap.get(DcMotor.class, "FrontRight"),
                hardwareMap.get(DcMotor.class, "FrontLeft"),
                hardwareMap.get(DcMotor.class, "BackRight"),
                hardwareMap.get(DcMotor.class, "BackLeft"));
        Thread t1 = new Thread(task, "t1");


        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");

        float power2 = 0;
        float power = 0;
        float power3 = 0;
        int Red;
        int Blue;
        boolean pressed = true;

        if (useExpanasionHub) {
            extendArm = hardwareMap.get(DcMotor.class, "extendArm");
            liftArm = hardwareMap.get(DcMotor.class, "liftArm");
            angleArm = hardwareMap.get(DcMotor.class, "angleArm");
        }

        TouchSen = hardwareMap.get(TouchSensor.class, "TouchSen");
        color = hardwareMap.get(ColorSensor.class, "Color");
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper2 = hardwareMap.get(CRServo.class, "gripper2");
        color = hardwareMap.get(ColorSensor.class, "Color");
        gripper.resetDeviceConfigurationForOpMode();
        gripper2.resetDeviceConfigurationForOpMode();


        waitForStart();
        t1.start();  //make sure to start thread
        if (opModeIsActive()) {
            while (opModeIsActive()) {


                Red = color.red();
                Blue = color.blue();


                if (gamepad2.left_trigger > 0) // if left trigger > 0
                {
                    power2 = gamepad2.left_trigger;
                    gamepad2.rumble(100);
                } else // check rtrigger
                {
                    power2 = -gamepad2.right_trigger;
                    gamepad2.rumble(100);
                }

                if (useExpanasionHub) {
                    liftArm.setPower(power2);
                }

                // true red
                if (Red > 500) {

                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }

                // less red
                if (Red > 250) {

                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                }


                // true blue
                if (Blue > 500) {

                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }


                // less blue
                if (Blue > 250) {

                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                }

                // less yellow
                if (Red > 250) {

                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                }

                // true yellow
                if (Red > 250) {

                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }


                //extra arm
                power = 0;
                power = gamepad2.right_stick_y;
                if (useExpanasionHub) {
                    extendArm.setPower(power);
                }

                //lift arm
                if (TouchSen.isPressed()) {
                    power = 0;
                } else {
                    power = gamepad2.left_stick_y;
                }

                if (useExpanasionHub) {
                    extendArm.setPower(power);
                }

                power3 = 0;
                power3 = gamepad2.left_stick_y;
                if (useExpanasionHub) {
                    angleArm.setPower(power3);
                }

                double open = 1;
                //Servo
                if (gamepad2.a) {
                    gripper.setPosition(open);
                    gripper2.setPower(open);
                }

                double close = 0;

                if (gamepad2.b) {
                    gripper.setPosition(close);
                    gripper2.setPower(close);
                }

                telemetry.addData("power", power);
                telemetry.addData("power2", power2);
                telemetry.addData("power3", power3);
                telemetry.addData("Touched", TouchSen.getValue());
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();


            }
        }

    }
}
//*********************************************
//  W A R N I N G   O U T D A T E D   D R I V E
//*********************************************
/*
    public void DoWork() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        int hex_motor_ticks;
        float vertical;
        float horizontal;
        float pivot;
        float power2 = 0;
        float power;
        float power3;
        int Red;
        boolean pressed = true;


        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        Arm3 = hardwareMap.get(DcMotor.class, "Arm3");
        TouchSen = hardwareMap.get(TouchSensor.class, "TouchSen");
        color = hardwareMap.get(ColorSensor.class, "Color");

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        // BackLeft.setDirection(DcMotor.Direction.REVERSE);
        // FrontRight.setDirection(DcMotor.Direction.REVERSE);
        // FrontLeft.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Red = color.red();


                if (gamepad2.left_trigger > 0) // if left trigger > 0
                {
                    power2 = gamepad2.left_trigger;
                    gamepad2.rumble(100);
                } else // check rtrigger
                {
                    power2 = -gamepad2.right_trigger;
                    gamepad2.rumble(100);
                }
                Arm2.setPower(power2);


                if (Red > 500) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    telemetry.addData("red see", Red);
                }


                if (gamepad2.b = pressed) {

                }


                //extra arm
                // power = gamepad2.right_stick_y;
                // Arm.setPower(power);

                //lift arm
                if (TouchSen.isPressed()) {
                    power = 0;
                } else {
                    power = gamepad2.left_stick_y;
                }
                Arm.setPower(power);

                power3 = gamepad2.right_stick_y;
                Arm3.setPower(power3);

                vertical = gamepad1.right_stick_x;
                horizontal = gamepad1.left_stick_x;
                pivot = gamepad1.left_stick_y;
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.8);
                BackRight.setPower((-pivot + vertical + horizontal) * 0.8);
                FrontLeft.setPower((pivot + vertical + horizontal) * 0.8);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 0.8);
                BackLeft.setTargetPosition((int) 0.5);
                telemetry.addData("power1", power);
                telemetry.addData("Touched", TouchSen.getValue());
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();



            }
        }
    }
}

 */

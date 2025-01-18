package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp
public class DeepWithMacro extends BaseAuto {
   // RevBlinkinLedDriver blinkinLedDriver;
    private Servo gripper;
    private CRServo gripper2;

    private TouchSensor TouchSen;

    private TouchSensor magSen;
    private ColorSensor color;

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;

    private DcMotor BackRight;

    private DcMotor extendArm;
    private DcMotor liftArm;
    private DcMotor angleArm;

    private boolean useExpanasionHub = true;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void RunOpModeInnerLoop() {

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


       // blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");

        float power2 = 0;
        float power = 0;
        float power3 = 0;
        float power4 = 0;
        int Red;
        int Blue;
        int Green;
        boolean pressed = true;
        int hex_motor_ticks = 288;


        extendArm = hardwareMap.get(DcMotor.class, "extendArm");
        liftArm = hardwareMap.get(DcMotor.class, "liftArm");
        angleArm = hardwareMap.get(DcMotor.class, "angleArm");


        TouchSen = hardwareMap.get(TouchSensor.class, "TouchSen");
        magSen = hardwareMap.get(TouchSensor.class, "magSen");
        color = hardwareMap.get(ColorSensor.class, "Color");
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper2 = hardwareMap.get(CRServo.class, "gripper2");
        color = hardwareMap.get(ColorSensor.class, "Color");
        gripper.resetDeviceConfigurationForOpMode();
        gripper2.resetDeviceConfigurationForOpMode();


        waitForStart();
        t1.start();
        if (opModeIsActive()) {
            // t1.start();
            while (opModeIsActive()) {


                Red = color.red();
                Blue = color.blue();
                Green = color.green();


                if (gamepad1.left_trigger > 0) // if left trigger > 0
                {
                    power2 = gamepad1.left_trigger;
                    gamepad2.rumble(100);
                } else // check rtrigger
                {
                    power2 = -gamepad1.right_trigger;
                    gamepad2.rumble(100);
                }


                liftArm.setPower(power2);


                // true red
                if (Red > 2000) {

                  //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }

                // less red
                // if (Red > 1000) {

                //   blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                //}

                if (Red < 800 & Blue < 800) {
                  //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }


                // true blue
                if (Blue > 2000) {

                  //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }


                // less blue
                //if (Blue > 1000) {

                //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                //}

                // less yellow
                //  if (Red > 250) {

                //    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                //}

                // true yellow
                //if (Red > 250) {

                //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                //}


                //extra arm
                power = 0;
                power = -gamepad2.right_stick_y;
                extendArm.setPower(power);


                // touch sen
                if (TouchSen.isPressed() || magSen.isPressed()) {
                    power = 0;
                }


                extendArm.setPower(power);


                power3 = 0;
                power3 = gamepad2.left_stick_y;

                angleArm.setPower(power3);

                power4 = 0;


                if (gamepad2.left_trigger > 0) {
                    //power4 = gamepad2.left_trigger;
                    //gamepad2.rumble(100);
                    gripper2.setPower(1);
                } else if (gamepad2.right_trigger > 0) {
                    gripper2.setPower(-1);
                } else {
                    gripper2.setPower(0);
                }
                if (gamepad2.a) {
                    extendPos = 0;
                    liftPos = 0;
                    anglePos = 0;
                    FrontRightPos = 0;
                    BackRightPos = 0;
                    FrontLeftPos = 0;
                    BackLeftPos = 0;
                    arm(5, 0, 0, 1);
                }


                if (gamepad2.b) {
                    extendPos = 0;
                    liftPos = 0;
                    anglePos = 0;
                    FrontRightPos = 0;
                    BackRightPos = 0;
                    FrontLeftPos = 0;
                    BackLeftPos = 0;
                    arm(0, 0, hex_motor_ticks * 5, 1);
                }


                telemetry.addData("power", power);
                telemetry.addData("power2", power2);
                telemetry.addData("power3", power3);
                telemetry.addData("Touched", TouchSen.getValue());
                telemetry.addData("Touched Magnet", magSen.getValue());
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();
            }
        }
    }
}

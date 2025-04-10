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
public class AnotherDeepDrive extends LinearOpMode {


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

    private boolean useExpanasionHub = true;

  private int extendPos = 0;
  private int liftPos = 0;
  private int anglePos = 0;
  private int FrontRightPos = 0;
  private int BackRightPos = 0;
  private int FrontLeftPos = 0;
  private int BackLeftPos = 0;

  private boolean Moving = false;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //DoWork();
        DoWork2();
    }
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
            Moving = true;
            // Do nothing
        }
        extendArm.setPower(0);
        liftArm.setPower(0);
        angleArm.setPower(0);
    }



    public void DoWork2() {
        BackGroundMech task = new BackGroundMech(gamepad1,
                hardwareMap.get(DcMotor.class, "FrontRight"),
                hardwareMap.get(DcMotor.class, "FrontLeft"),
                hardwareMap.get(DcMotor.class, "BackRight"),
                hardwareMap.get(DcMotor.class, "BackLeft"));

        Thread t1 = new Thread(task, "t1");


        //  blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");

        float power2 = 0;
        float power = 0;
        float power3 = 0;
        float power4 = 0;
        float climbPower = 0;
        int Red;
        int Blue;
        int Green;
        boolean pressed = true;


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
            while (opModeIsActive()) {





                if (!Moving) {


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


                    //extra arm
                    power = 0;
                    power = -gamepad2.right_stick_y;
                    extendArm.setPower(power);


                    // touch sen
                    // if (TouchSen.isPressed() || magSen.isPressed()) {
                    //    power = 0;
                    // }


                    extendArm.setPower(power);


                    power3 = 0;
                    power3 = gamepad2.left_stick_y;

                    angleArm.setPower(power3);

                    power4 = 0;


                    if (gamepad2.left_trigger > 0) // if left trigger > 0
                    {
                        //power4 = gamepad2.left_trigger;
                        //gamepad2.rumble(100);
                        gripper2.setPower(-1);
                        gripper.setPower(1);
                    } else if (gamepad2.right_trigger > 0) {
                        gripper2.setPower(1);
                        gripper.setPower(-1);
                    } else {
                        gripper2.setPower(0);
                        gripper.setPower(0);
                    }
                    if (gamepad2.a) {

                    }

                    climbArm.setPower(climbPower);
                    if (gamepad2.right_bumper) {
                        climbArm.setPower(1);
                    } else if (gamepad2.left_bumper) {
                        climbArm.setPower(-1);
                    } else climbArm.setPower(0);

                }

                    boolean MoveEncode = true;
                    if (Math.abs(power) > 0 || Math.abs(power2) > 0 || Math.abs(power3) > 0)
                    {
                        MoveEncode = false;
                    }

                    if (MoveEncode) {
                        if (gamepad2.a) {
                            arm(0, 1000, 0, 0);
                        }

                        if (gamepad2.b) {
                            arm(0, -1000, 0, 0);
                        }
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

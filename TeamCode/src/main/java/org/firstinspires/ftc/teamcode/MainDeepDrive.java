package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "1MainDeepDrive")
public class MainDeepDrive extends LinearOpMode {


    //RevBlinkinLedDriver blinkinLedDriver;



    private boolean Moving = false;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

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
                hardwareMap.get(DcMotor.class, "extendArmSideways"),
                hardwareMap.get(DcMotor.class, "extendArmUp"),
                hardwareMap.get(Servo.class, "OrientServo"),
                hardwareMap.get(Servo.class, "LevelServo"),
                hardwareMap.get(CRServo.class, "gripper"),
                hardwareMap.get(CRServo.class, "gripper2"),
                hardwareMap.get(ColorSensor.class, "Color"),
                hardwareMap.get(DcMotor.class, "DumpArm"),
                hardwareMap.get(TouchSensor.class, "touch"),
                Moving,
                hardwareMap.get(DcMotor.class, "climbArm"));

        Thread t2 = new Thread(task2, "t2");



        //port 2 3 ****************************************************
       /*
        gripper = hardwareMap.get(CRServo.class, "gripper");
        gripper2 = hardwareMap.get(CRServo.class, "gripper2");

        gripper.resetDeviceConfigurationForOpMode();
        gripper2.resetDeviceConfigurationForOpMode();


        */

        //TouchSensor touch = hardwareMap.get(TouchSensor.class, "touch");

        waitForStart();
        if (opModeIsActive()) {
            t1.start();
            t2.start();
            while (opModeIsActive()) {







                /*
                if (gamepad2.right_trigger > 0) {
                    gripper2.setPower(gamepad2.right_trigger);
                    gripper.setPower(-gamepad2.right_trigger);
                }


                   else if (Red > 2000) {
                        gripper2.setPower(0);
                        gripper.setPower(0);

                    } else if (Blue > 2000) {
                        gripper2.setPower(1);
                        gripper.setPower(-1);

                    } else {
                        gripper2.setPower(0);
                        gripper.setPower(0);
                    }

                 */

                task2.AddTelemetry(telemetry);
              //  telemetry.addData("touch=", touch.isPressed());
                telemetry.update();




            }
        }

    }
}
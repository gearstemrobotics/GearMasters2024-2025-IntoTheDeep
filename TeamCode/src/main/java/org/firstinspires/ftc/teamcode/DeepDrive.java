package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp(name = "DeepDrive (Blocks to Java)")
public class DeepDrive extends LinearOpMode {


    RevBlinkinLedDriver blinkinLedDriver;

    private TouchSensor TouchSen;
    private ColorSensor color;

    private DcMotor BackLeft;
   private DcMotor FrontRight;
    private DcMotor FrontLeft;

    private DcMotor BackRight;

    private DcMotor Arm;
    private DcMotor Arm2;
    private DcMotor Arm3;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        int hex_motor_ticks;
        int Right_Arm;
        int Left_Arm;
        float vertical;
        float horizontal;
        float pivot;
        float power2 = 0;
        float power;
        float power3;
        int Red;


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
                }
                else // check rtrigger
                {
                    power2 = -gamepad2.right_trigger;
                    gamepad2.rumble(100);
                }
               Arm2.setPower(power2);




                if (Red > 500)
                {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    telemetry.addData("red see", Red);
                }







                //extra arm
               // power = gamepad2.right_stick_y;
               // Arm.setPower(power);

                //lift arm
                if (TouchSen.isPressed())
                {
                    power = 0;
                }
                else
                {
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

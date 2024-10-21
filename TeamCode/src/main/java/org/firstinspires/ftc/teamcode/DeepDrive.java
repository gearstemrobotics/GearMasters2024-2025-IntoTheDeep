package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "DeepDrive (Blocks to Java)")
public class DeepDrive extends LinearOpMode {
    private TouchSensor touchSen;

    private DcMotor BackLeft;
  // private DcMotor FrontRight;
    private DcMotor FrontLeft;
    //private DcMotor rightarm;
    //private CRServo thingy1066;
    //private Servo plane;
    private DcMotor BackRight;
    //private DcMotor leftarm;
    //private DcMotor intake;
    //private CRServo thingy351;
    private DcMotor Arm;
    //private DcMotor Arm2;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int hex_motor_ticks;
        int Right_Arm;
        int Left_Arm;
        float vertical;
        float horizontal;
        float pivot;
        float power2 = 0;
        float power = 0;


        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
       // FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
     //   Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        touchSen = hardwareMap.get(TouchSensor.class, "touchSen");

        //BackRight.setDirection(DcMotor.Direction.REVERSE);
        //BackLeft.setDirection(DcMotor.Direction.REVERSE);
      //  FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);




        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                gamepad1.rumble(10000);

                if (gamepad1.left_trigger > 0) // if left trigger > 0
                {
                    power = gamepad1.left_trigger;
                    gamepad1.rumble(10000);
                }
                else // check rtrigger
                {
                    power = -gamepad1.right_trigger;
                    gamepad1.rumble(10000);
                }
                if (touchSen.isPressed())
                {
                    power = 0;
                }




               Arm.setPower(power);

                if (gamepad1.left_trigger > 0) // if left trigger > 0
                {
                    power2 = gamepad1.left_trigger;
                    gamepad1.rumble(100);
                }
                else // check rtrigger
                {
                    power2 = -gamepad2.right_trigger;
                    gamepad1.rumble(100);
                }
                if (touchSen.isPressed())
                {
                    power2 = 0;
                }
               // Arm2.setPower(power2);

                vertical = -gamepad1.right_stick_y;
                horizontal = gamepad1.right_stick_x;
                pivot = gamepad1.left_stick_x;
              //  FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.8);
                BackRight.setPower((-pivot + vertical + horizontal) * 0.8);
                FrontLeft.setPower((pivot + vertical + horizontal) * 0.8);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 0.8);
                BackLeft.setTargetPosition((int) 0.5);
                telemetry.addData("power1", power);
                telemetry.addData("Touched", touchSen.getValue());
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DeepDrive (Blocks to Java)")
public class DeepDrive extends LinearOpMode {

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    //private DcMotor rightarm;
    //private CRServo thingy1066;
    //private Servo plane;
    private DcMotor BackRight;
    //private DcMotor leftarm;
    //private DcMotor intake;
    //private CRServo thingy351;

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

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                vertical = -gamepad1.right_stick_y;
                horizontal = gamepad1.right_stick_x;
                pivot = gamepad1.left_stick_x;
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.8);
                BackRight.setPower((-pivot + vertical + horizontal) * 0.8);
                FrontLeft.setPower((pivot + vertical + horizontal) * 0.8);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 0.8);
                BackLeft.setTargetPosition((int) 0.5);
            }
        }
    }
}

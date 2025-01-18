package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DeepTest extends LinearOpMode {

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;

    private DcMotor BackRight;


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


        BackRight.setDirection(DcMotor.Direction.REVERSE);

        hex_motor_ticks = 288;
        Right_Arm = 0;
        Left_Arm = 0;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    BackRight.setPower(1);
                }

                if (gamepad1.b) {
                    BackLeft.setPower(1);
                }

                if (gamepad1.x) {
                    FrontLeft.setPower(1);
                }

                if (gamepad1.y) {
                    FrontRight.setPower(1);
                }
            }
        }
    }
}
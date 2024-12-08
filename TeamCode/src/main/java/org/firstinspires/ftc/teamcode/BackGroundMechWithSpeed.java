package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BackGroundMechWithSpeed implements Runnable {
    private boolean isRunning = true;

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;

    private DcMotor BackRight;
    private Gamepad GP;

    //All motors
    public BackGroundMechWithSpeed(Gamepad gamepad, DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft) {
        GP = gamepad;
        FrontRight = frontRight;
        FrontLeft = frontLeft;
        BackRight = backRight;
        BackLeft = backLeft;

    }


    @Override
    public void run() {
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        while (isRunning) {
// Do the work

            float vertical = GP.right_stick_x;
            float horizontal = GP.left_stick_x;
            float pivot = GP.left_stick_y;

            if (GP.a)
            {
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.5);
                BackRight.setPower((-pivot + vertical + horizontal) * 0.5);
                FrontLeft.setPower((pivot + vertical + horizontal) * 0.5);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 0.5);
            }
            else if (GP.b)
            {
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 1);
                BackRight.setPower((-pivot + vertical + horizontal) * 1);
                FrontLeft.setPower((pivot + vertical + horizontal) * 1);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 1);
            }
            else
            {
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 1);
                BackRight.setPower((-pivot + vertical + horizontal) * 1);
                FrontLeft.setPower((pivot + vertical + horizontal) * 1);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 1);
            }






            //BackLeft.setTargetPosition((int) 0.5);
        }

        // Stop running
    }

    public void stop() {
        isRunning = false;
    }
}
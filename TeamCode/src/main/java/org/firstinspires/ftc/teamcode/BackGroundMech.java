package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BackGroundMech implements Runnable {
    private boolean isRunning = true;

    private DcMotor BackLeft;

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;
    private Gamepad GP;

    //All motors
    public BackGroundMech(Gamepad gamepad, DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft) {
        GP = gamepad;
        FrontRight = frontRight;
        FrontLeft = frontLeft;
        BackRight = backRight;
        BackLeft = backLeft;

    }


    @Override
    public void run() {
         FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
       // BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        while (isRunning) {
// Do the work

            float pivot = -GP.right_stick_x;
            float horizontal = GP.left_stick_x;
            float vertical = -GP.left_stick_y;

            if (GP.left_bumper)
            {
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.5);
                BackRight.setPower((-pivot + vertical + horizontal) * 0.5);
                FrontLeft.setPower((pivot + vertical + horizontal) * 0.5);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 0.5);
            }
            else if (GP.right_bumper)
            {
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.25);
                BackRight.setPower((-pivot + vertical + horizontal) * 0.25);
                FrontLeft.setPower((pivot + vertical + horizontal) * 0.25);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 0.25);
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
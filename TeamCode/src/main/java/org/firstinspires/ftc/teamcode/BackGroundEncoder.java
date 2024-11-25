package org.firstinspires.ftc.teamcode;

import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BackGroundEncoder implements Runnable {
    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    int FrontRightPos;
    int LeftArmPos;
    int BackRightPos;
    int RightArmPos;
    int FrontLeftPos;
    int BackLeftPos;

    boolean isRunning;

    @Override
    public void run() {


       /* private void drive(double FrontRightTarget, double BackRightTarget, double FrontLeftTarget, double BackLeftTarget, double Speed) {
            FrontRightPos += FrontRightTarget;
            BackRightPos += BackRightTarget;
            FrontLeftPos += FrontLeftTarget;
            BackLeftPos += BackLeftTarget;
            FrontRight.setTargetPosition(FrontRightPos);
            BackRight.setTargetPosition(BackRightPos);
            FrontLeft.setTargetPosition(FrontLeftPos);
            BackLeft.setTargetPosition(BackLeftPos);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setPower(Speed);
            BackRight.setPower(Speed);
            FrontLeft.setPower(Speed);
            BackLeft.setPower(Speed);

        */
        while (isRunning) {
        }
    }
}

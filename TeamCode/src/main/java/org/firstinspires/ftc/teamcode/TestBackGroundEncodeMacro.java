package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TestBackGroundEncodeMacro implements Runnable {
    private boolean isRunning = true;

    private DcMotor liftArm;

    private DcMotor angleArm;
    private DcMotor extendArm;
    private Gamepad GP;
    private Gamepad GP2;


    private int extendPos;
    private int liftPos;
    private int anglePos;
    public boolean Moving = false;

    //All motors
    public TestBackGroundEncodeMacro(Gamepad gamepad2, Gamepad gamepad1, DcMotor LiftArm, DcMotor AngleArm, DcMotor ExtendArm, boolean moving) {
        Moving = moving;
        GP2 = gamepad2;
        GP = gamepad1;
        liftArm = LiftArm;
        angleArm = AngleArm;
        extendArm = ExtendArm;


    }

    public void arm(double ExtendArmTarget, double LiftArmTarget, double AngleArmTarget, double Speed) {
        Moving = true;
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
        while (isRunning && (extendArm.isBusy() || liftArm.isBusy() || angleArm.isBusy())) {
            // Do nothing
            Moving = true;
        }
        extendArm.setPower(0);
        liftArm.setPower(0);
        angleArm.setPower(0);
        Moving = false;
    }


    @Override
    public void run() {


        while (isRunning) {
// Do the work


            if (GP2.a) {
                arm(0, 1000, 0, 0);
            }



           else if (GP2.b) {
                arm(0, -1000, 0, 0);
            }
           else
           {
               double power2;
               if (GP.left_trigger > 0) // if left trigger > 0
               {
                   power2 = GP.left_trigger;
               } else // check rtrigger
               {
                   power2 = -GP.right_trigger;
               }


               liftArm.setPower(power2);


               //extra arm
               extendArm.setPower(-GP2.right_stick_y);

               angleArm.setPower(GP2.left_stick_y);


           }
            //BackLeft.setTargetPosition((int) 0.5);
        }

        // Stop running
    }

    public void stop() {
        isRunning = false;
    }
}
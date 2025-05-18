package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TestBackGroundEncodeMacro implements Runnable {
    private boolean isRunning = true;

    private CRServo gripper;
    private CRServo gripper2;
    private Servo OrientServo;
    private Servo LevelServo;
    private DcMotor liftArm;

    private DcMotor DumpArm;
    private DcMotor extendArmUp;
    private DcMotor extendArmSideways;
    private Gamepad GP;
    private Gamepad GP2;


    private int extendPos;
    private int liftPos;
    private int anglePos;
    public boolean Moving = false;

    //All motors
    public TestBackGroundEncodeMacro(Gamepad gamepad2, Gamepad gamepad1, DcMotor LiftArm, DcMotor AngleArm, DcMotor ExtendArm,
                                     Servo orientServo, Servo levelServo, CRServo Gripper, CRServo Gripper2, boolean moving) {
        Moving = moving;
        GP2 = gamepad2;
        GP = gamepad1;
        liftArm = LiftArm;
        extendArmUp = AngleArm;
        extendArmSideways = ExtendArm;
        OrientServo = orientServo;
        LevelServo = levelServo;
        gripper = Gripper;
        gripper2 = Gripper2;


    }

    public void arm(double ExtendArmUpTarget, double ExtendArmSidewaysTarget, double AngleArmTarget, double Speed) {
        Moving = true;
        extendPos += ExtendArmUpTarget;
        liftPos += ExtendArmSidewaysTarget;
        anglePos += AngleArmTarget;
        extendArmUp.setTargetPosition(extendPos);
        liftArm.setTargetPosition(liftPos);
        extendArmSideways.setTargetPosition(anglePos);
        extendArmUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmUp.setPower(Speed);
        liftArm.setPower(Speed);
        extendArmSideways.setPower(Speed);
        while (isRunning && (extendArmUp.isBusy() || liftArm.isBusy() || extendArmSideways.isBusy())) {
            // Do nothing
            Moving = true;
        }
        extendArmUp.setPower(0);
        liftArm.setPower(0);
        extendArmSideways.setPower(0);
        Moving = false;
    }


    @Override
    public void run() {



        while (isRunning) {

// Do the work

            //go down grab macro
            if (GP2.x)
            {
                
                arm(0, 1000, 0, 1);
                OrientServo.setPosition(1);
            }

            //go back up dispense and bring up macro
            if (GP2.y)
            {
                OrientServo.setPosition(0);
                arm(0, -1000, 0, 1);
                gripper.setPower(1);
                gripper2.setPower(-1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm(1000, 0, 0, 1);

            }


            if (GP2.a) {
                arm(0, 0, 100, 1);
            }



           else if (GP2.b) {
                arm(0, -1000, 0, 1);
            }
           else
           {
               liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               extendArmSideways.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               double power2;
               if (GP2.left_trigger > 0) // if left trigger > 0
               {
                   power2 = GP2.left_trigger;
               } else // check rtrigger
               {
                   power2 = -GP2.right_trigger;
               }


               liftArm.setPower(power2);


               //extra arm
               extendArmUp.setPower(-GP2.right_stick_y);

               extendArmSideways.setPower(GP2.left_stick_y);


           }
            //BackLeft.setTargetPosition((int) 0.5);
        }

        // Stop running
    }

    public void stop() {
        isRunning = false;
    }
}
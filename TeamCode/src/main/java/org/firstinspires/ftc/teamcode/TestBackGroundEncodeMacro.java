package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBackGroundEncodeMacro implements Runnable {
    private boolean isRunning = true;

    private TouchSensor touch;
    private CRServo gripper;
    private CRServo gripper2;
    private Servo OrientServo;
    private Servo LevelServo;
    //private DcMotor liftArm;

    private DcMotor climbArm;
    private DcMotor DumpArm;
    private DcMotor extendArmUp;
    private DcMotor extendArmSideways;
    private Gamepad GP;
    private Gamepad GP2;
    private ColorSensor color;

    private int extendPos;
    private int liftPos;
    private int anglePos;
    public boolean Moving = false;

    //All motors
    public TestBackGroundEncodeMacro(Gamepad gamepad2, Gamepad gamepad1, DcMotor AngleArm, DcMotor ExtendArm,
                                     Servo orientServo, Servo levelServo, CRServo Gripper, CRServo Gripper2, ColorSensor Color,
                                     DcMotor dumpArm, TouchSensor Touch, boolean moving, DcMotor ClimbArm) {
        Moving = moving;
        GP2 = gamepad2;
        GP = gamepad1;
       // liftArm = LiftArm;
        extendArmUp = AngleArm;
        extendArmSideways = ExtendArm;
        OrientServo = orientServo;
        LevelServo = levelServo;
        gripper = Gripper;
        gripper2 = Gripper2;
        color = Color;
        DumpArm = dumpArm;
        touch = Touch;
        climbArm = ClimbArm;


    }

    public void arm(double ExtendArmUpTarget, double ExtendArmSidewaysTarget, double Speed) {
      //  extendArmSideways.setDirection(DcMotorSimple.Direction.REVERSE);
        extendArmUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DumpArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Moving = true;
        anglePos += ExtendArmUpTarget;
        extendPos += ExtendArmSidewaysTarget;
       // liftPos += LiftArmTarget;
        extendArmUp.setTargetPosition(anglePos);
        //liftArm.setTargetPosition(liftPos);
        extendArmSideways.setTargetPosition(extendPos);
        extendArmUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmUp.setPower(Speed);
        //liftArm.setPower(Speed);
        extendArmSideways.setPower(Speed);
        while (isRunning && (extendArmUp.isBusy()  || extendArmSideways.isBusy())) {
            // Do nothing
            Moving = true;
            if (GP2.x)
            {
                break;
            }
        }
        extendArmUp.setPower(0);
        //liftArm.setPower(0);
        extendArmSideways.setPower(0);
        Moving = false;
    }

    class TelemData
    {
        public String s;
        public Object object;
    }

    private String _telemName = "EncoderMacro";
    private String _telemData = "";

    public void AddTelemetry(Telemetry telemetry)
    {
        telemetry.addData(_telemName, _telemData);
    }


    @Override
    public void run() {



        while (isRunning) {
            extendArmUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extendArmSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           // liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // to-do add rest of motors

// Do the work
            int Red;
            int Blue;
            int Green;

            Red = color.red();
            Blue = color.blue();
            Green = color.green();

            //go down grab macro
           // if (GP2.x)
            //{

              //  arm(0, 100, 0, 1);
               // OrientServo.setPosition(1);
                //LevelServo.setPosition(1);
           // }

            //go back up dispense and bring up macro
            if (GP2.y)
            {
                OrientServo.setPosition(1);
                LevelServo.setPosition(1);
                _telemData = "starting extend arm";
                boolean touched;
                if (touch.isPressed())
                {
                    touched = true;
                }
                else
                {
                    touched = false;
                }
                if (!touched) {
                    while (isRunning) {
                        if (touch.isPressed())
                        {
                            touched = true;
                        }
                        else
                        {
                            touched = false;
                        }
                        _telemData = "touch sensor pressed = " + touch.isPressed();
                        extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArmUp.setPower(-1);
                        if (GP2.x || touched) {
                            break;
                        }
                    }
                }
                _telemData = "stopped extend arm";
                extendArmSideways.setPower(0);
                extendArmUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                gripper.setPower(1);
                gripper2.setPower(-1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                }
                //arm(0, -100, 0, 1);

               /*
                gripper.setPower(1);
                gripper2.setPower(-1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm(1000, 0, 0, 1);

                */
                gripper.setPower(0);
                gripper2.setPower(0);
                //DumpArm.setPower(0.3);
                arm(5 ,2,1);
                // DumpArm.setPower(0);
              //  arm(8,0,1);

            }

/*
           else if (GP2.a) {
                arm(100, 0,  1);
            }


 */

            /*
           else if (GP2.b) {
                arm(100, 0, 0, 1);
            }

             */
           else
           {
              // liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               extendArmSideways.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                /*
               double power2;
               if (GP2.left_trigger > 0) // if left trigger > 0
               {
                   power2 = GP2.left_trigger;
               } else // check rtrigger
               {
                   power2 = -GP2.right_trigger;
               }

                 */

               if (GP2.left_trigger > 0) {
                   gripper2.setPower(-1);
                   gripper.setPower(1);
               } else if (GP2.right_trigger > 0) {
                   gripper2.setPower(1);
                   gripper.setPower(-1);

               }else if (Red > 440 ) {
                   //&& Green > 230 && Blue > 90) {
                   gripper2.setPower(-1);
                   gripper.setPower(1);

               }
               /*
               //Blue check
               else if (Blue > 500) {
                   gripper2.setPower(1);
                   gripper.setPower(-1);
               }

                */
               else
               {
                   gripper2.setPower(0);
                   gripper.setPower(0);
               }


               // liftArm.setPower(power2);
               if (GP2.dpad_up)
               {
                   OrientServo.setPosition(1);
                   LevelServo.setPosition(1);
               }

               if (GP2.dpad_down)
               {
                   OrientServo.setPosition(0);
                   LevelServo.setPosition(0);
               }



               //extra arm
               extendArmUp.setPower(-GP2.right_stick_y);

               extendArmSideways.setPower(GP2.left_stick_y);
               /*
               if (GP2.dpad_left)
               {
                   DumpArm.setPower(0.25);
               }
               if (GP2.dpad_right)
               {
                   DumpArm.setPower(-0.25);
               }
               if (GP2.x)
               {
                   DumpArm.setPower(0);
               }
               double DumpPower;


                */
               DumpArm.setPower(GP2.left_stick_x * 0.5);
               climbArm.setPower(GP2.right_stick_x * 0.5);


           }
            //BackLeft.setTargetPosition((int) 0.5);
        }

        // Stop running
    }

    public void stop() {
        isRunning = false;
    }
}
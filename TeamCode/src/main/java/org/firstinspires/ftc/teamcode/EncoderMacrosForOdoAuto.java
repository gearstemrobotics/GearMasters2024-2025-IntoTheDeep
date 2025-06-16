package org.firstinspires.ftc.teamcode;

import android.text.method.MovementMethod;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class EncoderMacrosForOdoAuto implements Runnable {

    private boolean isRunning = true;

    private TouchSensor touch;
    private TouchSensor touch2;
    private CRServo gripper;
    private CRServo gripper2;
    private Servo OrientServo;
    private Servo LevelServo;

    private DcMotor climbArm;
    private DcMotor DumpArm;
    private DcMotor extendArmUp;
    private DcMotor extendArmSideways;
    private ColorRangeSensor color;

    private int extendPos;
    private int anglePos;
    private static ElapsedTime myStopWatch = new ElapsedTime();

    public EncoderMacrosForOdoAuto(DcMotor ExtendArmSideways, DcMotor ExtendArmUp,
                                   Servo orientServo, Servo levelServo, CRServo Gripper, CRServo Gripper2, ColorRangeSensor Color,
                                   DcMotor dumpArm, TouchSensor Touch, TouchSensor Touch2, DcMotor ClimbArm) {
        extendArmSideways = ExtendArmSideways;
        extendArmUp = ExtendArmUp;
        OrientServo = orientServo;
        LevelServo = levelServo;
        gripper = Gripper;
        gripper2 = Gripper2;
        color = Color;
        DumpArm = dumpArm;
        touch2 = Touch2;
        touch = Touch;
        climbArm = ClimbArm;

    }

    public boolean hasBlock() {
        int Green = color.green();
        int Blue = color.blue();
        int Red = color.red();
        double Distance = color.getDistance(DistanceUnit.MM);
        return (Distance < 27);
       // return (Red > 300 && Green > 170 && Blue > 100);// || Red > 15 && Green > 160 && Blue > 150)
    }

    public void arm(double ExtendArmUpTarget, double ExtendArmSidewaysTarget, double Speed) {
        extendArmUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        anglePos += ExtendArmUpTarget;
        extendPos += ExtendArmSidewaysTarget;
        extendArmUp.setTargetPosition(anglePos);
        extendArmSideways.setTargetPosition(extendPos);
        extendArmUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArmUp.setPower(Speed);
        extendArmSideways.setPower(Speed);

        while (isRunning && (extendArmUp.isBusy() || extendArmSideways.isBusy())) {
        }

        extendArmUp.setPower(0);
        extendArmSideways.setPower(0);
    }

    class TelemData {
        public String s;
        public Object object;
    }

    private String _telemName = "EncoderMacro";
    private String _telemData = "";

    public void AddTelemetry(Telemetry telemetry) {
        telemetry.addData(_telemName, _telemData);
    }

    public enum Operation {
        None,
        MoveArmIn,
        MoveArmUp,
        MoveArmOut,
        MoveArmDown
    }

    private volatile Operation _operation = Operation.None;

    public Operation GetCurrentOperation() {
        return _operation;
    }

    public void CompleteOperation() {
        while (_operation != EncoderMacrosForOdoAuto.Operation.None) {
            // wait for the operation to end or the program to stop
            if (!isRunning) break;

            try {
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                break;
            }
        }
    }

    public boolean DoOperation(Operation operation) {
        // cannot do operation if one is in progress
        if (_operation != Operation.None) return false;

        _operation = operation;
        return true;
    }

    public void MoveArmOut() {
        extendArmUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // arm(0, 100, 1);
        extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OrientServo.setPosition(0);
        LevelServo.setPosition(0);

        while (true) {
            if (hasBlock()) {
                gripper.setPower(0);
                gripper2.setPower(0);
                extendArmSideways.setPower(0);
                break;
            } else {
                gripper.setPower(-1);
                gripper2.setPower(1);
                extendArmSideways.setPower(0.5);
            }
        }
    }


    public void MoveArmDown() {
        extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myStopWatch.reset();
        boolean touchy = false;
        while (!touchy) {
              DumpArm.setPower(-0.35);
            extendArmUp.setPower(-1);
            if (touch2.isPressed())
            {
                touchy = true;
            }
        }
        DumpArm.setPower(0);
        extendArmUp.setPower(0);
    }

    public void MoveArmUp() {
        extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // DumpArm.setPower(0.26 );
        myStopWatch.reset();

        while (myStopWatch.seconds() < 2.4) {
            extendArmUp.setPower(1);
        }

        while(myStopWatch.seconds() < 3.3)
        {
            DumpArm.setPower(0.6 );
        }

        extendArmUp.setPower(0);
        DumpArm.setPower(0);
    }

    public boolean isTouched() {
        if (touch.isPressed()) {
            return true;
        } else {
            return false;
        }
    }

    public void MoveArmIn() {
        extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OrientServo.setPosition(1);
        LevelServo.setPosition(1);
        //_telemData = "starting extend arm";

        // move arm in until touch sensor is reached
        while (isRunning && !isTouched()) {
            extendArmSideways.setPower(-1);
        }

        gripper.setPower(1);
        gripper2.setPower(-1);

        // sleep for a second
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ignored) {
        }

        gripper.setPower(0);
        gripper2.setPower(0);
    }

    @Override
    public void run() {

        while (isRunning) {
            switch (_operation) {
                case MoveArmIn:
                    MoveArmIn();
                    _operation = Operation.None;
                    break;
                case MoveArmUp:
                    MoveArmUp();
                    _operation = Operation.None;
                    break;
                case MoveArmOut:
                    MoveArmOut();
                    _operation = Operation.None;
                    break;
                case MoveArmDown:
                    MoveArmDown();
                    _operation = Operation.None;
                    break;
                case None:
                default:
                    break;
            }
        }
    }

    public void stop() {
        isRunning = false;
    }
}


//**********************************************************

   /*

            if (GP2.y) {
                OrientServo.setPosition(1);
                LevelServo.setPosition(1);
                _telemData = "starting extend arm";
                boolean touched;
                if (touch.isPressed()) {
                    touched = true;
                } else {
                    touched = false;
                }
                if (!touched) {
                    while (isRunning) {
                        if (touch.isPressed()) {
                            touched = true;
                        } else {
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

                gripper.setPower(0);
                gripper2.setPower(0);

            }


            if (GP2.left_trigger > 0) {
                gripper2.setPower(-1);
                gripper.setPower(1);
            } else if (GP2.right_trigger > 0) {
                gripper2.setPower(1);
                gripper.setPower(-1);

            } else {
                gripper2.setPower(0);
                gripper.setPower(0);
            }


            // liftArm.setPower(power2);
            if (GP2.dpad_up) {
                OrientServo.setPosition(1);
                LevelServo.setPosition(1);
            }

            if (GP2.dpad_down) {
                OrientServo.setPosition(0);
                LevelServo.setPosition(0);
            }

*/

//BackLeft.setTargetPosition((int) 0.5);


// Stop running


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


public class EncoderMacrosForOdoAuto implements Runnable {

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


    public EncoderMacrosForOdoAuto(DcMotor ExtendArmSideways, DcMotor ExtendArmUp,
                                   Servo orientServo, Servo levelServo, CRServo Gripper, CRServo Gripper2, ColorSensor Color,
                                   DcMotor dumpArm, TouchSensor Touch, DcMotor ClimbArm) {
        extendArmSideways = ExtendArmSideways;
        extendArmUp = ExtendArmUp;
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
        while (isRunning && (extendArmUp.isBusy() || extendArmSideways.isBusy())) {
            // Do nothing
            Moving = true;
            if (GP2.x) {
                break;
            }
        }
        extendArmUp.setPower(0);
        //liftArm.setPower(0);
        extendArmSideways.setPower(0);
        Moving = false;
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
        MoveArmUp;
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

    public void MoveArmIn() {
        extendArmUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArmSideways.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OrientServo.setPosition(1);
        LevelServo.setPosition(1);
        //_telemData = "starting extend arm";
        boolean touched;
        if (touch.isPressed()) {
            touched = true;
        } else {
            touched = false;
        }
        if (!touched) {
            while (isRunning && !touched) {
                if (touch.isPressed()) {
                    touched = true;
                } else {
                    touched = false;
                    extendArmSideways.setPower(-1);
                    extendArmUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }
            }
            gripper.setPower(1);
            gripper2.setPower(-1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
            }

            gripper.setPower(0);
            gripper2.setPower(0);
        }
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
                    extendArmSideways.setPower(-1);
                    _operation = Operation.None;
                    break;
                case None:
                default:
                    break;
            }
        }
    }
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


    public void stop() {
        isRunning = false;
    }
}

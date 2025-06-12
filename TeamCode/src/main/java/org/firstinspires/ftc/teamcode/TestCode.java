package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Locale;

@TeleOp(name = "Test code")

public class TestCode extends LinearOpMode {
    private DcMotor BackLeft;

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;
    private GoBildaPinpointDriver odo;
    // private DcMotor Arm;


    @Override
    public void runOpMode() {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //odo.setOffsets(-83.5, 12.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(-85.0, 12.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        float power;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                odo.update();
                if (gamepad1.a){
                    odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
                }


                Pose2D pos = odo.getPosition();
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
                String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                telemetry.addData("Velocity", velocity);
                BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
                FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
/*
                if (gamepad1.x)
                {
                    FrontLeft.setPower(0.2); // forward
                }
                if (gamepad1.y)
                {
                    FrontRight.setPower(0.2); // forward
                }
                if (gamepad1.a)
                {
                    // port 0
                    BackLeft.setPower(0.2); // forward
                }
                if (gamepad1.b)
                {
                    BackRight.setPower(0.2); // forward
                }
               /* BackLeft.setPower(0.1);
                BackRight.setPower(0.3);
                FrontLeft.setPower(0.7);
                FrontRight.setPower(1);


                */
                float pivot = gamepad1.right_stick_x;
                float horizontal = gamepad1.left_stick_x;
                float vertical = -gamepad1.left_stick_y;

                if (gamepad1.left_bumper)
                {
                    FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.5);
                    BackRight.setPower((-pivot + vertical + horizontal) * 0.5);
                    FrontLeft.setPower((pivot + vertical + horizontal) * 0.5);
                    BackLeft.setPower((pivot + (vertical - horizontal)) * 0.5);
                }
                else if (gamepad1.right_bumper)
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


                telemetry.update();
            }
        }
    }
}
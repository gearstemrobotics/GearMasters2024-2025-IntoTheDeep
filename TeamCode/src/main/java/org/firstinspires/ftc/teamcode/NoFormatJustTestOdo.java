package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@Autonomous
public class NoFormatJustTestOdo extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DcMotor BackLeft;

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;

    private void simpleForward(double goal, double speed)
    {
        odo.resetPosAndIMU();

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double yGoalSet = 2;
        double modifyedX = odo.getPosition().getX(DistanceUnit.INCH);;
        modifyedX -= goal;

        double Y = modifyedX;
        double X = odo.getPosition().getY(DistanceUnit.INCH);
        double Yaw = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double power1 = 0;
        double power2 = 0;
        double power3 = 0;
        double seeNoPower = 0;
        double Xlimit = 2;
        double Yawlimit = 2;

        boolean isStrafe = false;
        boolean isFor = false;
        boolean isTurn = false;
        String setStatus = "nothing";

        while (opModeIsActive()){
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);
            telemetry.update();


        // forward back X
        if (X < Xlimit && X > -Xlimit) {
            power1 = 0;
            isFor = false;
        } else if (X > Xlimit) {
            //move back
            power1 = 0.2;
            isFor = true;
            setStatus = "Forward";
        } else if (X < -Xlimit) {
            // move forward
            power1 = -0.2;
            isFor = true;
            setStatus = "Back";
        }

        //Turn Yaw
        if (Yaw < Yawlimit && Yaw > -Yawlimit) {
            power2 = 0;
        } else if (Yaw >= Yawlimit) {
            power2 = 0.2;
            isTurn = true;
            setStatus = "turn";
        } else if (Yaw <= -Yawlimit) {
            power2 = -0.2;
            isTurn = true;
            setStatus = "Turn";
        }

        //Strafe Y
        if ( isFor || isTurn) {
            power3 = 0;
        } else if (Y >= yGoalSet) {
            power3 = -0.3;
            isStrafe = true;
            setStatus = "Target";
        } else if (Y <= yGoalSet -1) {
            power3 = 0.3;
            isStrafe = true;
            setStatus = "Target";

        } else {
            power3 = 0;

        }


        //Power 1 = 1st set For/ Back
        if (isFor) {
            FrontLeft.setPower(power1);
            BackLeft.setPower(-power1);
            BackRight.setPower(power1);
            FrontRight.setPower(-power1);
        }

        //power 2 = second set turn
        else if (isTurn) {
            FrontLeft.setPower(-power2);
            BackLeft.setPower(-power2);
            BackRight.setPower(power2);
            FrontRight.setPower(power2);
        }

        //power 3 = third set that strafe to target
        else if (isStrafe) {
            FrontLeft.setPower(power3);
            BackLeft.setPower(power3);
            BackRight.setPower(power3);
            FrontRight.setPower(power3);
        }

        // seeNoPower = nothing is see just goes in circles
        else {
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            FrontRight.setPower(0);
        }
        }
    }

    @Override

    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
       FrontRight.setDirection(DcMotor.Direction.REVERSE);
        // BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        odo.setOffsets(-83.5, 12.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);



        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //odo.resetPosAndIMU();
                //simpleForward(10, 0);
               // break;

                simpleForward(10,1);
            }
        }
    }
}

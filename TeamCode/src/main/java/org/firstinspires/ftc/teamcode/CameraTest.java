package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Autonomous()
public class CameraTest extends BaseAuto {

    private CameraMonitor cameraMonitor;
    public CameraTest()
    {
    }

    int LoopCount = 0;

    @Override
    protected void Map()
    {
        super.Map(); // call base class map first


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        cameraMonitor = new CameraMonitor(webcamName);
        Thread t1 = new Thread(cameraMonitor, "t1");
        t1.start();
    }

    private void AddTelemtry() {
        telemetry.addData("state", state.toString());
        telemetry.addData("x(calc)", String.format("%.2f", cameraMonitor.GetCalculatedX()));
        telemetry.addData("x", String.format("%.2f", cameraMonitor.GetX()));
        telemetry.addData("y", String.format("%.2f", cameraMonitor.GetY()));
        telemetry.addData("z", String.format("%.2f", cameraMonitor.GetZ()));
        telemetry.addData("yaw", String.format("%.2f", cameraMonitor.GetYaw()));
        telemetry.addData("range", String.format("%.2f", cameraMonitor.GetRange()));
        telemetry.addData("bearing", String.format("%.2f", cameraMonitor.GetBearing()));
        telemetry.addData("LoopCount", LoopCount);
        telemetry.update();
    }

    public enum HomingState
    {
        WaitForCamera,
        ScanForTag,
        HeadToTag,
        CenterOnTag,
        FineTune,
    }

    HomingState state = HomingState.WaitForCamera;

    @Override
    protected void RunOpModeInnerLoop() {
        // kickoff thread for camera here
        boolean hasJumped = false;

        while (opModeIsActive())
        {
            LoopCount++;
            AddTelemtry();
            telemetry.update();
            if (cameraMonitor.IsReady()) {
                //AprilTagHoming();
                if (state == HomingState.WaitForCamera)
                {
                    Sleep(2000);
                    state = HomingState.ScanForTag;
                }
                else if (state == HomingState.ScanForTag)
                {
                    scanForTag();
                    // wait a bit to make sure tag refreshes
                    Sleep(500);

                    // only stop scanning if we found it
                    if (cameraMonitor.GetPose() != null) {
                        state = HomingState.HeadToTag;
                    }
                }
                else if (state == HomingState.HeadToTag) {
                    GoToAprilTag();
                    if (cameraMonitor.GetRange() < 15 || Math.abs(cameraMonitor.GetYaw()) > 55)
                    {
                        state = HomingState.CenterOnTag;

                        // wait a bit to make sure tag refreshes
                        Sleep(500);
                    }
                }
                else if (state == HomingState.CenterOnTag)
                {
                    JumpToAprilTag();
                    if (Math.abs(cameraMonitor.GetYaw()) < 2)
                    {
                        state = HomingState.FineTune;

                        // wait a bit to make sure tag refreshes
                        Sleep(500);
                    }
                }
                else if (state == HomingState.FineTune)
                {
                    AprilTagHoming();

                    if (IsHomed())
                    {
                        // break out of the loop, we are done
                        break;

                        /*
                        // lets see how we did!
                        Sleep(5000);
                        // rotate 180 and kick off the search again
                        drive(-1000,-1000,1000,10000,0.5);
                        state = HomingState.ScanForTag;
                        */
                    }

                }
            }
        }
    }

    public void Sleep(long milliseconds)
    {
        try { Thread.sleep(milliseconds); } catch (InterruptedException e) { }
    }

    public boolean IsHomed()
    {
        AprilTagPoseFtc pose = cameraMonitor.GetPose();
        if (pose != null) {

            double y = pose.y;
            double yaw = Math.abs(pose.yaw);
            if (y < 13 && y > 11 && yaw < 4) {
                return true;
            }
        }
        return false;
    }

    private void scanForTag() {
        int hex_motor_ticks = 56;
        while (true) {
            if (cameraMonitor.GetPose() != null) {
                return;
            }

            for (int i = 0; i < 10; i++) {
                drive(-hex_motor_ticks * 2, -hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.5);
                if (cameraMonitor.GetPose() != null) {
                    return;
                }
            }

            for (int i = 0; i < 20; i++) {
                drive(hex_motor_ticks * 2, hex_motor_ticks * 2, -hex_motor_ticks * 2, -hex_motor_ticks * 2, 0.5);
                if (cameraMonitor.GetPose() != null) {
                    return;
                }
            }
        }
    }

    private boolean AprilTagHoming() {
        // switch to power mode
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double Y = cameraMonitor.GetY();
        double X = cameraMonitor.GetX();
        double Yaw = cameraMonitor.GetYaw();
        double power1 = 0;
        double power2 = 0;
        double power3 = 0;
        double seeNoPower = 0;
        double Xlimit = 5;
        double Yawlimit = 4;

        boolean isStrafe = false;
        boolean isFor = false;
        boolean isTurn = false;
        String setStatus = "nothing";


        // forward back X
        if (Double.isNaN(X)) {
            power1 = 0;
            isFor = false;
        } else if (X >= Xlimit) {
            //move back
            power1 = 0.3;
            isFor = true;
            setStatus = "Forward";
        } else if (X < -Xlimit) {
            // move forward
            power1 = -0.3;
            isFor = true;
            setStatus = "Back";
        } else {
            power1 = 0;
            isFor = false;
        }

        //Turn Yaw
        if (Double.isNaN(Yaw) || isFor) {
            power2 = 0;
        } else if (Yaw >= Yawlimit) {
            power2 = 0.3;
            isTurn = true;
            setStatus = "turn";
        } else if (Yaw <= -Yawlimit) {
            power2 = -0.3;
            isTurn = true;
            setStatus = "Turn";
        } else {
            power2 = 0;
            isTurn = false;
        }


        //Strafe Y
        if (Double.isNaN(Y) || isFor || isTurn) {
            power3 = 0;
        } else if (Y >= 13) {
            power3 = -0.3;
            isStrafe = true;
            setStatus = "Target";
        } else if (Y <= 11) {
            power3 = 0.3;
            isStrafe = true;
            setStatus = "Target";

        } else {
            power3 = 0;

        }

        if (Double.isNaN(Y) && Double.isNaN(X) && Double.isNaN(Yaw)) {
            // zeroout turn and
            seeNoPower = 0.1;
        } else {
            seeNoPower = 0;
        }
        //Power 1 = 1st set For/ Back
        if (isFor) {
            FrontLeft.setPower(power1);
            BackLeft.setPower(power1);
            BackRight.setPower(power1);
            FrontRight.setPower(power1);
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
            BackLeft.setPower(-power3);
            BackRight.setPower(power3);
            FrontRight.setPower(-power3);
        }

        // seeNoPower = nothing is see just goes in circles
        else {
            FrontLeft.setPower(seeNoPower);
            BackLeft.setPower(seeNoPower);
            BackRight.setPower(-seeNoPower);
            FrontRight.setPower(-seeNoPower);


        }
        //telemetry.addData(setStatus, "status");
       // telemetry.update();
        // try {
        // Thread.sleep(50);
        // } catch (InterruptedException e) {
        //   throw new RuntimeException(e);
        // }

        return !(Double.isNaN(Y) && Double.isNaN(X) && Double.isNaN(Yaw));
    }

    void JumpToAprilTag()
    {
        double Y = cameraMonitor.GetY();
        Y = Y - 14;//32;
        double X = cameraMonitor.GetCalculatedX();

        double Yaw = cameraMonitor.GetYaw();
        int hex_motor_ticks = 288;
        int turnYaw = (int)(Yaw * -13.5);
        int forX = (int)(X * 65.0);
        int strafeY = (int)(Y * 65.0);
        // rotate by yaw
        drive(-turnYaw * 1,-turnYaw * 1,turnYaw * 1,turnYaw * 1,0.3);

        // move in X
        //drive(hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, hex_motor_ticks * 2, 0.3);
        drive(forX * 1,forX * 1,forX * 1,forX * 1,0.3);
        // move in Y

          drive(-strafeY * 1,strafeY * 1,strafeY * 1,-strafeY * 1,0.3);
    }

    void GoToAprilTag()
    {
        AprilTagPoseFtc pose = cameraMonitor.GetPose();
        if (pose != null) {
            double range = pose.range;
            range = range - 16;

            double bearing = pose.bearing;
            int hex_motor_ticks = 288;
            int turn = (int) (bearing * -13.5);
            int strafe = (int) (range * -40.0);

            // rotate
            drive(-turn * 1, -turn * 1, turn * 1, turn * 1, 0.3);

            // strafe
            drive(-strafe * 1, strafe * 1, strafe * 1, -strafe * 1, 0.3);
        }
    }

}
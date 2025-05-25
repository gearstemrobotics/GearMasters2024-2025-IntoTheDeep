package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class BaselineAuto extends BaseAuto{



    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override
    protected void RunInit() {

        super.RunInit();

        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto
        super.RunInit();
    }
        // Run Auto if stop was not pressed.
        @Override
        public void RunOpModeInnerLoop() {

        int hex_motor_ticks = 288;
        robot.turnTo(90, 0.45, 0.5);

        // arm(0, hex_motor_ticks * 12, 0, 0.7);
       // Home();
        //drive(-hex_motor_ticks * 9 , -hex_motor_ticks * 9, hex_motor_ticks * 9, hex_motor_ticks * 9, 1);
    }




        }


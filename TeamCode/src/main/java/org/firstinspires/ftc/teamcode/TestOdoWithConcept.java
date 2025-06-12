package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * High-precision example OpMode demonstrating advanced odometry navigation
 */
@Autonomous
public class TestOdoWithConcept extends LinearOpMode {

    private SimpleNavigationSystem navigation = new SimpleNavigationSystem();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the navigation system
        telemetry.addData("Status", "Initializing High-Precision Navigation...");
        telemetry.update();

        navigation.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready! Place robot at starting position");
        telemetry.addData("Info", "Precision square pattern - 0.25\" accuracy");
        telemetry.addData("", "Press start when robot is positioned exactly");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            // High-precision square pattern with smaller, more precise movements

            // Move to position 1: 18 inches forward with high precision
            telemetry.addData("Status", "Precision move to position 1...");
            telemetry.update();

            navigation.resetPosition();

            move(10, 0, 0);
            move(10, 0, 90);
            move(0, 0, 0);
            move(0,0,180);
            move(0,10,0);
            move(0,0,0);

            move(0,0,-90);
            move(0,0,0);
            /*
            move(10, 0, 0);
            move(0, 0, 0);
            //navigation.resetPosition();
            move(-10, 0, 0);
            move(0, 0, 0);
            //navigation.resetPosition();
            move(0, 10, 0);
            move(0, 0, 0);
            //navigation.resetPosition();
            move(0, -10, 0);
            move(0, 0, 0);


             */
            //navigation.resetPosition();

            // Move to position 2: 18 inches right, turn to face movement direction
            telemetry.addData("Status", "Precision move to position 2...");
            telemetry.update();

            //return;
/*
            while (opModeIsActive() && !navigation.moveUntilPositioned(18, 18, 90)) {
                telemetry.update();
                sleep(10);
            }
            sleep(1000);

            // Move to position 3: back to Y=0, face backward
            telemetry.addData("Status", "Precision move to position 3...");
            telemetry.update();

            while (opModeIsActive() && !navigation.moveUntilPositioned(18, 0, 180)) {
                telemetry.update();
                sleep(10);
            }
            sleep(1000);

            // Move back to start with precision
            telemetry.addData("Status", "Precision return to start...");
            telemetry.update();

            while (opModeIsActive() && !navigation.moveUntilPositioned(0, 0, 270)) {
                telemetry.update();
                sleep(10);
            }

            // Final precision alignment to original heading
            telemetry.addData("Status", "Final precision alignment...");
            telemetry.update();

            while (opModeIsActive() && !navigation.moveUntilPositioned(0, 0, 0)) {
                telemetry.update();
                sleep(10);
            }

            // Precision test complete - show final accuracy
            telemetry.addData("=== PRECISION TEST COMPLETE ===", "");
            telemetry.addData("Final Position Error", "X=%.3f\", Y=%.3f\"",
                    navigation.getCurrentX(),
                    navigation.getCurrentY());
            telemetry.addData("Final Heading Error", "%.2f degrees",
                    navigation.getCurrentHeading());
            telemetry.addData("", "Errors should be < 0.25\" and < 1°");
            telemetry.update();

            // Hold position for observation
            while (opModeIsActive()) {
                // Continue showing position for manual verification
                if (navigation.moveUntilPositioned(10, 0, 0)) break;
                telemetry.update();
                sleep(50);
            }

 */
        }



        // Stop the navigation system
        navigation.stop();
    }

    public void move(double x, double y, double heading)
    {
        while (opModeIsActive() && !navigation.moveUntilPositioned(x, y, heading)) {
            telemetry.update();
            sleep(10); // Faster update for precision
        }
        sleep(1000); // Longer pause to observe precision
    }
}

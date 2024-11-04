package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp (name = "Test code")

public class TestCode extends LinearOpMode {
    private ColorSensor Color;
    private ColorSensor Color2;
    private TouchSensor TouchSen;
    private DcMotor Arm;
    @Override
    public void runOpMode() {
        float power;
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        TouchSen = hardwareMap.get(TouchSensor.class, "TouchSen");
        Color = hardwareMap.get(ColorSensor.class, "Color");
        Color2 = hardwareMap.get(ColorSensor.class, "Color2");
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (TouchSen.isPressed())
                {
                    power = 0;
                }
                else
                {
                    power = gamepad2.left_stick_y;
                }
                Arm.setPower(power);

                telemetry.addData("Red", Color.red());
                telemetry.addData("Green", Color.green());
                telemetry.addData("Blue", Color.blue());


                telemetry.addData("Red2", Color2.red());
                telemetry.addData("Green2", Color2.green());
                telemetry.addData("Blue2", Color2.blue());
                telemetry.update();
            }
        }
    }
}
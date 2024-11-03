package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp()
public class HelloMotor extends OpMode {

    private DcMotor Arm;
    private ColorSensor color;
    private DistanceSensor distance;
    private TouchSensor touchSen;
    @Override
    public void init()
    {
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        touchSen = hardwareMap.get(TouchSensor.class, "TouchSen");
        color = hardwareMap.get(ColorSensor.class, "Color");
        distance =  hardwareMap.get(DistanceSensor.class, "distance");
    }

    @Override
    public void loop() {
        float power = 0;
        if (gamepad1.left_trigger > 0) // if left trigger > 0
        {
            power = gamepad1.left_trigger;
        }
        else // check rtrigger
        {
            power = -gamepad1.right_trigger;
        }
        if (touchSen.isPressed())
        {
            power = 0;
        }
        Arm.setPower(power);
        telemetry.addData("power1", power);
        telemetry.addData("Touched", touchSen.getValue());
        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
    }
}

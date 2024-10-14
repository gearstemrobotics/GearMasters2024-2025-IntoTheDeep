package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class HelloMotor extends OpMode {

    private DcMotor rightMotor;
    @Override
    public void init()
    {
        rightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        rightMotor.setPower(power);
        telemetry.addData("power1", power);
    }
}

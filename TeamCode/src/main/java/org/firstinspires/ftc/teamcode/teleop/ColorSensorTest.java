package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;

@TeleOp(group = "competition")
@Config
public class ColorSensorTest extends LinearOpMode {
    BrushlandColorSensor sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new BrushlandColorSensor(0, "color", hardwareMap);
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("p0", sensor.getPin0());
            telemetry.addData("p1", sensor.getPin1());
            telemetry.update();
        }
    }
}
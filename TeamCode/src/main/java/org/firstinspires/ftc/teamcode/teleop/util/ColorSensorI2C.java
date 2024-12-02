package org.firstinspires.ftc.teamcode.teleop.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;

@TeleOp(group = "competition")
@Config
public class ColorSensorI2C extends LinearOpMode {
    RevColorSensorV3 sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(RevColorSensorV3.class, "Color");
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            NormalizedRGBA color = sensor.getNormalizedColors();
            telemetry.addData("r", color.red);
            telemetry.addData("g", color.green);
            telemetry.addData("b", color.blue);
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
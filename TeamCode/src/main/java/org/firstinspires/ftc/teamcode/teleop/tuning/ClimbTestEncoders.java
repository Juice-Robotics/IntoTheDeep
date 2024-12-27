package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(group = "competition")
@Config
public class ClimbTestEncoders extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput climb1Encoder = hardwareMap.get(AnalogInput.class, "climb1Encoder");
        AnalogInput climb2Encoder = hardwareMap.get(AnalogInput.class, "climb2Encoder");
        Robot robot = new Robot(hardwareMap, false);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            double climb1 = climb1Encoder.getVoltage() / 3.3 * 360;
            double climb2 = climb2Encoder.getVoltage() / 3.3 * 360;

            telemetry.addData("climb1", climb1);
            telemetry.addData("climb2", climb2);

            if (gamepad1.dpad_up){
                robot.climbWinch.setPower(1);
            }
            else if (gamepad1.dpad_down){
                robot.climbWinch.setPower(-1);
            }else{
                robot.climbWinch.setPower(0);
            }

            telemetry.update();
        }
    }
}
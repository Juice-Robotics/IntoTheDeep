package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;

@TeleOp(group = "competition")
@Config
public class AutoL2Test extends LinearOpMode {
    ContinuousServo climb1;
    ContinuousServo climb2;

    double setupTargetPosition1;
    double setupTargetPosition2;

    double pullTargetPosition1;
    double pullTargetPosition2;

    private static final double kP = 0.01;
    private static final double kF = 0.1;

    private void setClimbPosition(ContinuousServo servo, double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        double p = error * kP;
        double f = Math.signum(error) * kF;
        double power = p + f;
        power = Math.max(-1, Math.min(1, power));
    }
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            climb1 = new ContinuousServo(0, "climb1", hardwareMap);
            climb2 = new ContinuousServo(0, "climb2", hardwareMap);

            AnalogInput climb1Encoder = hardwareMap.get(AnalogInput.class, "climb1Encoder");
            AnalogInput climb2Encoder = hardwareMap.get(AnalogInput.class, "climb2Encoder");
            // Initialize your own robot class
            waitForStart();
            if (isStopRequested()) return;
            while (opModeIsActive() && !isStopRequested()) {
                double climb1CurrPos = climb1Encoder.getVoltage() / 3.3 * 360;
                double climb2CurrPos = climb2Encoder.getVoltage() / 3.3 * 360;
                if (gamepad1.dpad_up) {
                    setClimbPosition(climb1, climb1CurrPos, setupTargetPosition1);
                    setClimbPosition(climb2, climb2CurrPos, setupTargetPosition2);
                }
                else if (gamepad1.dpad_down) {
                    setClimbPosition(climb1, climb1CurrPos, pullTargetPosition1);
                    setClimbPosition(climb2, climb2CurrPos, pullTargetPosition2);
                }

            }

            telemetry.update();
        }
}

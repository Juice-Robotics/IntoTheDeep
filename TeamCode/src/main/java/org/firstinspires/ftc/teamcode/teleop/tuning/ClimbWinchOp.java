package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

@TeleOp(group = "competition")
@Config
@Disabled
public class ClimbWinchOp extends LinearOpMode {
    ContinuousServo climb1;
    ContinuousServo climb2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        climb1 = new ContinuousServo(0, "climb1", hardwareMap);
        climb2 = new ContinuousServo(0, "climb2", hardwareMap);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.triangle) {
                climb1.setSpeed((float) -0.1);
                climb2.setSpeed((float) -0.1);
            } else if (gamepad1.cross) {
                climb1.setSpeed((float) 0.1);
                climb2.setSpeed((float) 0.1);
            } else {
                climb1.setSpeed(0);
                climb2.setSpeed(0);
            }
        }
    }
}


// INTAKE (DOWN)
// Arm: 103, elbow: 282, ext 190
// INTAKE (INTERMEDIATE)
// Arm: 130, elbow 282, ext 190
// INTERMEDIATE
// Arm: 260, Elbow: 190, Ext: 100

// LIFT MAX 2150

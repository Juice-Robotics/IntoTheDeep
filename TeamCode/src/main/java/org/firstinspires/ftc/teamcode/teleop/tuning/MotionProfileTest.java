package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.control.MotionProfile;
import org.firstinspires.ftc.teamcode.util.control.MotionProfileGenerator;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "competition")
@Config
@Disabled
public class MotionProfileTest extends LinearOpMode {
    public static double MAX_VEL = 0;
    public static double MAX_ACCEL = 0;
    public static int TARGET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(1,0, MAX_VEL, MAX_ACCEL);
        int oldTarget = 0;
        double position = 0;
        ElapsedTime timer = new ElapsedTime();
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (TARGET != oldTarget) {
                profile = MotionProfileGenerator.generateSimpleMotionProfile(position, TARGET, MAX_VEL, MAX_ACCEL);
                oldTarget = TARGET;
            }

            position = profile.get(timer.time(TimeUnit.SECONDS));
            telemetry.addData("POS", position);
            telemetry.addData("TARGET", TARGET);
        }
    }
}
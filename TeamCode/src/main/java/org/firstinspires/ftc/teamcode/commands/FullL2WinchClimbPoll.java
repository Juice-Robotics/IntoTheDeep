package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.Levels;

public class FullL2WinchClimbPoll implements Action {
    Robot robot;

    public FullL2WinchClimbPoll(Robot r) {
        robot = r;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if ((Math.abs(robot.climbWinch.servo1.getAngle() - 20) < 1)) {
            robot.state = Levels.CLIMB_L2;
            robot.climbWinch.setPower(0);
            return false;
        }
        return true;
    }
}

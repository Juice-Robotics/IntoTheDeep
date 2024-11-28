package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;
import org.firstinspires.ftc.teamcode.util.enums.Levels;

public class WinchStopAtPosition implements Action {
    ClimbWinch climbWinch;
    float ticks;

    public WinchStopAtPosition(ClimbWinch cW, float ticks) {
        climbWinch = cW;
        this.ticks = ticks;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (Math.abs(climbWinch.getPosition() - ticks) < 1) {
            climbWinch.setPower(0);
            return false;
        }
        return true;
    }
}
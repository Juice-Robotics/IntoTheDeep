package org.firstinspires.ftc.teamcode.zestyzoom.trajectory.commands;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.util.Command;
import org.firstinspires.ftc.teamcode.zestyzoom.Drive;

public class PointTrajectory implements Command {
    Drive drive;
    Pose2D endTarget;
    boolean started = false;

    public PointTrajectory(Drive drive, Pose2D target) {
        this.drive = drive;
        endTarget = target;
    }

    @Override
    public boolean run() {
        if (!started) {
            drive.setTarget(endTarget);
            started = true;
            return true;
        } else return !drive.isCloseTo(drive.pose, endTarget);
    }
}

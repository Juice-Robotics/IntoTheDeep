package org.firstinspires.ftc.teamcode.zestyzoom.trajectory.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.util.Command;
import org.firstinspires.ftc.teamcode.zestyzoom.Drive;

public class PointTrajectory implements Command {
    Drive drive;
    Pose2D endTarget;
    boolean started = false;
    Telemetry telemetry;

    public PointTrajectory(Drive drive, Pose2D target, Telemetry telemetry) {
        this.drive = drive;
        endTarget = target;
        this.telemetry = telemetry;

    }

    @Override
    public boolean run() {
        telemetry.addData("End x " , endTarget.getX(DistanceUnit.INCH));
        telemetry.addData("Cur x " , drive.pose.getX(DistanceUnit.INCH));
        telemetry.addData("Done" ,drive.isCloseTo(drive.pose, endTarget) );
        if (!started) {
            drive.setTarget(endTarget);
            started = true;
            return true;
        } else return !drive.isCloseTo(drive.pose, endTarget);
    }
}

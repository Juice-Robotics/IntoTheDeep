package org.firstinspires.ftc.teamcode.zestyzoom.trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.primitives.SleepCommand;
import org.firstinspires.ftc.teamcode.commands.util.Command;
import org.firstinspires.ftc.teamcode.zestyzoom.Drive;
import org.firstinspires.ftc.teamcode.zestyzoom.trajectory.commands.PointTrajectory;

import java.util.ArrayList;

public class TrajectoryBuilder {
    Pose2D startingPosition;
    Drive drive;
    Pose2D endPose;
    ArrayList<Command> trajectory = new ArrayList<>();
    public TrajectoryBuilder(Drive drive, Pose2D startingPos) {
        startingPosition = startingPos;
        endPose = startingPos;
        this.drive = drive;
    }

    public TrajectoryBuilder addPoint(Pose2D target) {
        trajectory.add(new PointTrajectory(drive, target));
        endPose = target;
        return this;
    }

    public TrajectoryBuilder waitSeconds(double time) {
        trajectory.add(new SleepCommand(time));
        return this;
    }

    public TrajectoryCommand build() {
        Command[] trajectoryList = trajectory.toArray(new Command[0]);
        return new TrajectoryCommand(startingPosition, endPose, trajectoryList);
    }
}

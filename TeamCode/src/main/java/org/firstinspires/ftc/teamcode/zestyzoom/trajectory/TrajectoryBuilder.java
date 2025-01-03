package org.firstinspires.ftc.teamcode.zestyzoom.trajectory;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.primitives.SleepCommand;
import org.firstinspires.ftc.teamcode.commands.util.Command;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.zestyzoom.Drive;
import org.firstinspires.ftc.teamcode.zestyzoom.trajectory.commands.PointTrajectory;

import java.util.ArrayList;
import java.util.Arrays;

public class TrajectoryBuilder {
    Pose2D startingPosition;
    Drive drive;
    Pose2D endPose;
    ArrayList<Command> trajectory = new ArrayList<>();
    Telemetry telemetry;
    TelemetryPacket packet = new TelemetryPacket();
    public TrajectoryBuilder(Drive drive, Pose2D startingPos, Telemetry telemetry) {
        startingPosition = startingPos;
        endPose = startingPos;
        this.drive = drive;
        this.telemetry = telemetry;

        packet.fieldOverlay().setStroke("blue");
        Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(startingPosition.getX(DistanceUnit.INCH), startingPosition.getY(DistanceUnit.INCH), startingPosition.getHeading(AngleUnit.RADIANS)));
    }

    public TrajectoryBuilder addPoint(Pose2D target) {
        packet.fieldOverlay()
                .setStroke("blue")
                .setStrokeWidth(5)
                .fillPolygon(new double[]{endPose.getX(DistanceUnit.INCH), target.getX(DistanceUnit.INCH)}, new double[]{endPose.getY(DistanceUnit.INCH), target.getY(DistanceUnit.INCH)});
        packet.fieldOverlay().setStroke("#34ad38");
        Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(target.getX(DistanceUnit.INCH), target.getY(DistanceUnit.INCH), target.getHeading(AngleUnit.RADIANS)));
        trajectory.add(new PointTrajectory(drive, target, telemetry));
        endPose = target;
        return this;
    }

    public TrajectoryBuilder turn(double angle) {
        trajectory.add(new PointTrajectory(drive, new Pose2D(DistanceUnit.INCH, endPose.getX(DistanceUnit.INCH), endPose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, endPose.getHeading(AngleUnit.RADIANS) + angle), telemetry));
        endPose = new Pose2D(DistanceUnit.INCH, endPose.getX(DistanceUnit.INCH), endPose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, endPose.getHeading(AngleUnit.RADIANS) + angle);
        return this;
    }

    public TrajectoryBuilder turnTo(double heading) {
        trajectory.add(new PointTrajectory(drive, new Pose2D(DistanceUnit.INCH, endPose.getX(DistanceUnit.INCH), endPose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, heading), telemetry));
        endPose = new Pose2D(DistanceUnit.INCH, endPose.getX(DistanceUnit.INCH), endPose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, heading);
        return this;
    }

    public TrajectoryBuilder waitSeconds(double time) {
        trajectory.add(new SleepCommand(time));
        return this;
    }

    public TrajectoryCommand build() {
        Command[] trajectoryList = trajectory.toArray(new Command[0]);
        return new TrajectoryCommand(startingPosition, endPose, packet, trajectoryList);
    }
}

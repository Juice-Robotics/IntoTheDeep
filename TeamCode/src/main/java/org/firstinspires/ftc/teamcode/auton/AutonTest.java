package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.primitives.LoopCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.ParallelCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.SequentialCommand;
import org.firstinspires.ftc.teamcode.commands.util.Commands;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.trajectory.TrajectoryCommand;

@Autonomous(name = "a", group = "Autonomous")
public class AutonTest extends LinearOpMode {
    public void runOpMode() {
        Pose2D beginPose = new Pose2D(DistanceUnit.INCH, -12.0, -60.0, AngleUnit.RADIANS, Math.PI);
        //Pose2D beginPose = new Pose2D(DistanceUnit.INCH, -12.0, -60.0, AngleUnit.RADIANS, 0);
        Drive drive = new Drive(hardwareMap, beginPose, telemetry);

        TrajectoryCommand preloadDrive = drive.trajectoryBuilder(drive.pose)
                //.addPoint(new Pose2D(DistanceUnit.INCH, 0, -29, AngleUnit.RADIANS, Math.PI))
                .addPoint(new Pose2D(DistanceUnit.INCH, 0, -40, AngleUnit.RADIANS, 0))
                .waitSeconds(1)
                .build();
        TrajectoryCommand spike1 = drive.trajectoryBuilder(preloadDrive.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH,-38, -43, AngleUnit.RADIANS, Math.toRadians(-23)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand observation1 = drive.trajectoryBuilder(spike1.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH,-35, -44.5, AngleUnit.RADIANS, Math.toRadians(-145)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand spike2 = drive.trajectoryBuilder(observation1.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH,-45.2, -42, AngleUnit.RADIANS, Math.toRadians(-10)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand observation2 = drive.trajectoryBuilder(spike2.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, -38, -44.5, AngleUnit.RADIANS, Math.toRadians(-150)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand back = drive.trajectoryBuilder(observation2.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, -21, -38, AngleUnit.RADIANS, Math.toRadians(-135)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand intake1 = drive.trajectoryBuilder(back.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, -27, -43, AngleUnit.RADIANS, Math.toRadians(-135)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand deposit = drive.trajectoryBuilder(intake1.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, -5, -29, AngleUnit.RADIANS, Math.PI))
                .waitSeconds(1)
                .build();
        TrajectoryCommand intake2 = drive.trajectoryBuilder(deposit.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, -25, -44,  AngleUnit.RADIANS, Math.toRadians(-135)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand deposit2 = drive.trajectoryBuilder(intake2.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, 0, -29, AngleUnit.RADIANS, Math.PI))
                .waitSeconds(1)
                .build();
        TrajectoryCommand intake3 = drive.trajectoryBuilder(deposit2.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, -25, -44,  AngleUnit.RADIANS, Math.toRadians(-135)))
                .waitSeconds(1)
                .build();
        TrajectoryCommand deposit3 = drive.trajectoryBuilder(intake3.endPose())
                .addPoint(new Pose2D(DistanceUnit.INCH, -3, -29, AngleUnit.RADIANS, Math.PI))
                .waitSeconds(1)
                .build();
        telemetry.addData("Status","starting");
        telemetry.update();
//        robot.initSubsystems();
        waitForStart();
        if (isStopRequested()) return;
        Commands.runBlocking(
            new ParallelCommand(
                new SequentialCommand(
                    // PRELOAD DEPOSIT
                    preloadDrive,
                    spike1,
                    observation1,
                    spike2,
                    observation2,
                    back,
                    intake1,
                    deposit,
                    intake2,
                    deposit2,
                    intake3,
                    deposit3

                ),
                new LoopCommand(drive::update, this::isStopRequested)
            )
        );
    }
}
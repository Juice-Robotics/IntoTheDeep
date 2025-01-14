package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.primitives.InstantCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.LoopCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.ParallelCommand;
import org.firstinspires.ftc.teamcode.commands.primitives.SequentialCommand;
import org.firstinspires.ftc.teamcode.commands.util.Commands;
import org.firstinspires.ftc.teamcode.zestyzoom.Drive;
import org.firstinspires.ftc.teamcode.zestyzoom.trajectory.TrajectoryCommand;

@Autonomous(name = "a", group = "Autonomous")
public class AutonTestBuckets extends LinearOpMode {
    public void runOpMode() {
        //Pose2D beginPose = new Pose2D(DistanceUnit.INCH, -12.0, -60.0, AngleUnit.RADIANS, Math.PI);
        Pose2D beginPose = new Pose2D(DistanceUnit.INCH, -12.0, -60.0, AngleUnit.RADIANS, 0);
        Drive drive = new Drive(hardwareMap, beginPose, telemetry);


        TrajectoryCommand preload = drive.trajectoryBuilder(drive.pose)
                //preload
                .addPoint(new Pose2d(-56, -56, Math.toRadians(45)))
                .waitSeconds(3)
                .build();
        TrajectoryCommand spike1 = drive.trajectoryBuilder(preload.endPose())
                //spike1
                .addPoint(new Pose2d(-48, -48, Math.toRadians(90)))
                .waitSeconds(1)

                .build();
        TrajectoryCommand depo1 = drive.trajectoryBuilder(spike1.endPose())
                //depo1
                .addPoint(new Pose2d(-56, -56, Math.toRadians(45)))
                .waitSeconds(3)

                .build();
        TrajectoryCommand spike2 = drive.trajectoryBuilder(depo1.endPose())
                //spike2
                .addPoint(new Pose2d(-58, -48, Math.toRadians(90)))
                .waitSeconds(1)

                .build();
        TrajectoryCommand depo2 = drive.trajectoryBuilder(spike2.endPose())
                //depo2
                .addPoint(new Pose2d(-56, -56, Math.toRadians(45)))
                .waitSeconds(3)

                .build();
        TrajectoryCommand spike3 = drive.trajectoryBuilder(depo2.endPose())
                //spike3
                .addPoint(new Pose2d(-54, -45, Math.toRadians(127)))
                .waitSeconds(1)

                .build();
        TrajectoryCommand depo3 = drive.trajectoryBuilder(spike3.endPose())
                //depo3
                .addPoint(new Pose2d(-56, -56, Math.toRadians(45)))
                .waitSeconds(3)

                .build();
        TrajectoryCommand park = drive.trajectoryBuilder(depo3.endPose())
                //ascent zone park
                .addPoint(new Pose2d(-48, -10, Math.toRadians(0)))
                .addPoint(new Pose2d(-25, -10, Math.toRadians(0)))

                .build();

        LoopCommand cmdLoop = new LoopCommand(drive::update, this::isStopRequested);
        telemetry.addData("Status","starting");
        telemetry.update();
//        robot.initSubsystems();
        waitForStart();
        if (isStopRequested()) return;
        //                            robot.lift.update();
        //                             PoseKeeper.set(robot.drive.pose);
        Commands.runBlocking(
                new ParallelCommand(
                        new SequentialCommand(
                                preload,
                                spike1,
                                depo1,
                                spike2,
                                depo2,
                                spike3,
                                depo3,
                                park,
                                new InstantCommand(cmdLoop::kill)
                        ),
                        cmdLoop
                )
        );
    }
}
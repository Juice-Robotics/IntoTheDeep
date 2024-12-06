package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LoopAction;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;

@Autonomous(name = "HPSideRedV1.1", group = "Autonomous")
public class HPSideRedV1_1 extends LinearOpMode {
    Robot robot;
    KalmanDrive drive;
    CVMaster cv;
    Limelight3A limelight;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(-90));
        PoseKeeper.set(beginPose);
        robot = new Robot(hardwareMap, true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cv = new CVMaster(limelight, null);
        drive = new KalmanDrive(hardwareMap, beginPose, limelight);

        Action preloadDrive = drive.actionBuilder(drive.pose)
                .setTangent(2.03444)
                .splineToLinearHeading(new Pose2d(0, -29, Math.toRadians(-90)), Math.toRadians(110))
                .waitSeconds(0.25)
                .build();
        telemetry.addData("is","starting");
        telemetry.update();
        robot.initSubsystems();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // PRELOAD DEPOSIT
                                new ParallelAction(
                                        preloadDrive,
                                        new SequentialAction(
//                                            new SleepAction(1),
                                            new InstantAction(robot::highRung)
                                        )
                                ),
//                                new SleepAction(0.5),
                                robot.outtakeSpecimen(true),
                                new InstantAction(() -> robot.lift.runToPosition(810)),
                                new SleepAction(0.2),
                                new InstantAction(robot::intermediatePreset)

//                                // SPIKE RIGHT
//                                new ParallelAction(
//                                        driveToSpikeR,
//                                        new SequentialAction(
//                                                new SleepAction(0.5),
//                                                robot.intakePreset(50, true)
//                                        )
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToObR,
//                                        new InstantAction(robot::preloadDropPreset)
//                                ),
//                                robot.outtakeSample(true),
//
//                                // SPIKE CENTER
//                                new ParallelAction(
//                                        driveToSpikeC,
//                                        robot.intakePreset(50, true)
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToObC,
//                                        new InstantAction(robot::preloadDropPreset)
//                                ),
//                                robot.outtakeSample(true),
//
//                                //SPIKE LEFT
//                                new ParallelAction(
//                                        driveToSpikeL,
//                                        robot.intakePreset(50, true)
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToObL,
//                                        new InstantAction(robot::preloadDropPreset)
//                                ),
//                                robot.outtakeSample(true),
//
//                                // **CYCLE #1**
//                                driveToObservation1,
//                                new InstantAction(() -> robot.intakePreset(50)),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToChamber1,
//                                        new SequentialAction(
//                                                new SleepAction(0),
//                                                new InstantAction(robot::highRung)
//                                        )
//                                ),
//                                robot.smartOuttake(true),
//
//                                // **CYCLE #2**
//                                new ParallelAction(
//                                        driveToObservation2,
//                                        new SequentialAction(
//                                                new SleepAction(0.1),
//                                                robot.intakePreset(50, true)
//                                        )
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToChamber2,
//                                        new SequentialAction(
//                                                new SleepAction(0),
//                                                new InstantAction(robot::highRung)
//                                        )
//                                ),
//                                robot.smartOuttake(true),
//
//                                // **CYCLE #3**
//                                new ParallelAction(
//                                        driveToObservation3,
//                                        new SequentialAction(
//                                                new SleepAction(0.1),
//                                                robot.intakePreset(50, true)
//                                        )
//                                ),
//                                robot.commands.stopIntake(SampleColors.RED),
//                                new ParallelAction(
//                                        driveToChamber3,
//                                        new SequentialAction(
//                                                new SleepAction(0),
//                                                new InstantAction(robot::highRung)
//                                        )
//                                ),
//                                robot.smartOuttake(true),
//
//                                // **PARK**
//                                new ParallelAction(
//                                        park,
//                                        new SequentialAction(
//                                                new InstantAction(robot::autonObParkPreset)
//                                        )
//                                )
                        ),
                        new LoopAction(() -> {
                            robot.lift.update();
//                            PoseKeeper.set(robot.drive.pose);
                        }, this::isStopRequested)
                )
        );
       // robot.cv.kill();
    }
}
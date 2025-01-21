package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseKeeper;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

@Autonomous(name = "BucketSide", group = "Autonomous")
public class BucketSide extends LinearOpMode {
    KalmanDrive drive;
    CVMaster cv;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-30, -60, Math.toRadians(0));
//        cv = new CVMaster(hardwareMap.get(Limelight3A.class, "limelight"), hardwareMap.get(WebcamName.class, "Webcam 1"));
//        cv.start();
//        cv.setLLPipeline(CVMaster.LLPipeline.APRILTAGS);

        drive = new KalmanDrive(hardwareMap, beginPose, cv.limelight);

        Action preload = drive.actionBuilder(drive.pose)
                //preload
                .setTangent(Math.toRadians(165))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(200))
                .waitSeconds(3)

                .build();
        Action spike1 = drive.actionBuilder(new Pose2d(-56, -56, Math.toRadians(45)))
                //spike1
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(90)), Math.toRadians(45))
                .waitSeconds(1)

                .build();
        Action depo1 = drive.actionBuilder(new Pose2d(-48, -48, Math.toRadians(90)))
                //depo1
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(225))
                .waitSeconds(3)

                .build();
        Action spike2 = drive.actionBuilder(new Pose2d(-56, -56, Math.toRadians(45)))
                //spike2
                .setTangent(Math.toRadians(92))
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(100))
                .waitSeconds(1)

                .build();
        Action depo2 = drive.actionBuilder(new Pose2d(-58, -48, Math.toRadians(90)))
                //depo2
                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(272))
                .waitSeconds(3)

                .build();
        Action spike3 = drive.actionBuilder(new Pose2d(-56, -56, Math.toRadians(45)))
                //spike3
                .setTangent(Math.toRadians(88))
                .splineToLinearHeading(new Pose2d(-54, -45, Math.toRadians(127)), Math.toRadians(92))
                .waitSeconds(1)

                .build();
        Action depo3 = drive.actionBuilder(new Pose2d(-54, -45, Math.toRadians(127)))
                //depo3
                .setTangent(Math.toRadians(268))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(268))
                .waitSeconds(3)

                .build();
        Action park = drive.actionBuilder(new Pose2d(-56, -56, Math.toRadians(45)))
                //ascent zone park
                .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(0)), Math.toRadians(0))

                .build();
        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        spike1,
                        depo1,
                        spike2,
                        depo2,
                        spike3,
                        depo3,
                        park
                ));
        PoseKeeper.set(drive.pose);
    }
}
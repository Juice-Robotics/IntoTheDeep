package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBucketAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16, -60, Math.toRadians(-90)))
                //preload
                .setTangent(Math.toRadians(165))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(200))
                .waitSeconds(3)

                //spike1
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(90)), Math.toRadians(45))
                .waitSeconds(1)

                //depo1
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(225))
                .waitSeconds(3)

                //spike2
                .setTangent(Math.toRadians(92))
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(100))
                .waitSeconds(1)

                //depo2
                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(272))
                .waitSeconds(3)

                //spike3
                .setTangent(Math.toRadians(88))
                .splineToLinearHeading(new Pose2d(-54, -45, Math.toRadians(127)), Math.toRadians(92))
                .waitSeconds(1)

                //depo3
                .setTangent(Math.toRadians(268))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(268))
                .waitSeconds(3)

                //ascent zone park
                .splineToLinearHeading(new Pose2d(-48, -10, Math.toRadians(0)), Math.toRadians(80))
                .setTangent(Math.toRadians(80))
                .splineToConstantHeading(new Vector2d(-25, -10), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

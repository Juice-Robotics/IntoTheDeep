package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingDepositForward {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(-90)))
                .setTangent(2.03444)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), Math.toRadians(110))

                .setReversed(true)
                .setTangent(Math.toRadians(-17))
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(70)), Math.toRadians(-17))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(-30)), Math.toRadians(-17))
                .waitSeconds(0.25)

                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(60)), 0)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -46, Math.toRadians(60)), 0)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(60)), Math.PI)
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(-30)), Math.PI)
                .waitSeconds(0.25)

                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(50)), 0)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, -46, Math.toRadians(50)), 0)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(50)), Math.PI)
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(-30)), Math.PI)

                .setTangent(2.89661)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 2.89661)
                .waitSeconds(0.1)
                .setTangent(-0.24497)
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(-30)), -0.24497)
                .waitSeconds(0.5)

                .setTangent(2.89661)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 2.89661)
                .waitSeconds(0.1)
                .setTangent(-0.24497)
                .splineToLinearHeading(new Pose2d(40, -46, Math.toRadians(-30)), -0.24497)
                .waitSeconds(0.5)

                .setTangent(2.89661)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 2.89661)
                .waitSeconds(0.1)

                .setTangent(-0.58800)
                .splineToLinearHeading(new Pose2d(36, -60, Math.toRadians(-90)), -0.58800)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

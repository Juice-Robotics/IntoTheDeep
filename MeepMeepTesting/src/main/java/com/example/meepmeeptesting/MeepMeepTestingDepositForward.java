package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
                //drive to sub (preload)
                .setTangent(2.03444)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), Math.toRadians(110))
                .waitSeconds(0.25)

                //drive to spike1
                .setReversed(true)
                .setTangent(Math.toRadians(-17))
                .splineToLinearHeading(new Pose2d(24, -48, Math.toRadians(45)), Math.toRadians(0))
                .waitSeconds(0.5)

                //turn to observation zone1
                .splineToLinearHeading(new Pose2d(24, -48, Math.toRadians(-30)), Math.toRadians(0))
                .waitSeconds(0.5)

                //drive to spike2
                .splineToLinearHeading(new Pose2d(36, -48, Math.toRadians(45)), Math.toRadians(0))
                .waitSeconds(0.5)

                //turn to observation zone2
                .splineToLinearHeading(new Pose2d(36, -48, Math.toRadians(-30)), Math.toRadians(0))
                .waitSeconds(0.5)

                //drive to spike3
                .splineToLinearHeading(new Pose2d(48, -48, Math.toRadians(45)), Math.toRadians(0))
                .waitSeconds(0.5)

                //back up + turn to observation zone3
                .splineToLinearHeading(new Pose2d(36, -48, Math.toRadians(45)), Math.toRadians(180)) //backup to not hit wall with extension
                .splineToLinearHeading(new Pose2d(36, -48, Math.toRadians(-45)), Math.toRadians(0)) //turns to observation
                .waitSeconds(0.5)

                //intake 1 specimen + drive to sub (cycle1)
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-45)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)

                //intake 1 specimen + drive to sub (cycle2)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-45)), -Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)

                //intake 1 specimen + drive to sub (cycle3)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-45)), -Math.PI/10)
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .setTangent(-Math.PI/10)

                //park
                .splineToLinearHeading(new Pose2d(17, -42, Math.toRadians(-45)), -Math.PI/10)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

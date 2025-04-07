package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -62, Math.toRadians(270)))
                /*.splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -44), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(49, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(49, -52))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(60, -60))
                .splineToConstantHeading(new Vector2d(48.25, -58), Math.toRadians(270))
                .turnTo(Math.toRadians(90))
                .lineToY(-63.25, new TranslationalVelConstraint(20.0)) */
                .lineToY(-36)
                        .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(36, -42), Math.toRadians(270))
                    .turnTo(Math.toRadians(150))
                    .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(50, -42))
                .turnTo(Math.toRadians(150))
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(60, -42))
                .turnTo(Math.toRadians(150))
                .turnTo(Math.toRadians(270))
                /*.splineToConstantHeading(new Vector2d(48.25, -50), Math.toRadians(270))
                .turnTo(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48.25, -58), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10, -36), Math.toRadians(270)) */
                .build()
        );




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
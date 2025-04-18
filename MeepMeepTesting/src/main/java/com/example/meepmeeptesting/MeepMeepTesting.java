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
                .setConstraints(60, 60, Math.toRadians(90), Math.toRadians(90), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-32.5, -62, Math.toRadians(270)))
                .splineToSplineHeading(new Pose2d(39, -16, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60, -11), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60, -50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(37, -56), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(37, -64), Math.toRadians(90))

                .build()
        );




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
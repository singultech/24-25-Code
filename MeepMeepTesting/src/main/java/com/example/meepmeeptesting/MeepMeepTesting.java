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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                .waitSeconds(0.2)
                .lineToY(-28)
                .waitSeconds(0.25)
                .lineToY(-41)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(280)), Math.toRadians(280))
                .strafeTo(new Vector2d(34, -43))
                .strafeTo(new Vector2d(37, -30))
                .turn(Math.toRadians(-190))
                //slides come back in
                .splineToSplineHeading(new Pose2d(39, -16, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -11), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(47, -56), Math.toRadians(90))
                //Grab here
//                .waitSeconds(0.5)
//                .lineToY(-28)
//                .waitSeconds(0.5)
//                .lineToY(-43)
//                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(240)), Math.toRadians(240))
//                .strafeTo(new Vector2d(37.5, -40.5))
//                .strafeTo(new Vector2d(38, -38))
//                .turnTo(Math.toRadians(140))
//                .turnTo(Math.toRadians(240))
//                .strafeTo(new Vector2d(44, -38))
                .build()
        );




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
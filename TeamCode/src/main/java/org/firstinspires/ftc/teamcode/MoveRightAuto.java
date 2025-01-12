package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.FrontGrabber;

@Autonomous(name = "Move Right")
public class MoveRightAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FrontGrabber grabber = new FrontGrabber(0.72, 1, hardwareMap);
        long curTime;
        long lastOpened = 0;
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder goRight = drive.actionBuilder(initialPose)
                        .lineToY(70);

        waitForStart();
Actions.runBlocking(
        drive.actionBuilder(new Pose2d(15,10,Math.toRadians(90)))
                .lineToY(30)
                .build());
    }
}


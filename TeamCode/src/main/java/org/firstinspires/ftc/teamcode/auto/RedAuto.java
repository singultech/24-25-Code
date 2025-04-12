package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(10, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .lineToY(-34)
                    .waitSeconds(1)
                    .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(240)), Math.toRadians(240))
                    .splineToConstantHeading(new Vector2d(36, -42), Math.toRadians(240))
                    .turnTo(Math.toRadians(150))
                    .strafeTo(new Vector2d(50, -42))
                    .turnTo(Math.toRadians(240))
                    .turnTo(Math.toRadians(150))
                .build());
    }
}
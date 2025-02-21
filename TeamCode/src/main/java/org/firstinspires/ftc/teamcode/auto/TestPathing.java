package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@Autonomous(name = "Pathing Tests", group = "Testing")
public class TestPathing extends LinearOpMode {
    @Override
    public void runOpMode() {
        Action trajectoryAction;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(10, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        TrajectoryActionBuilder testTraj = drive.actionBuilder(initialPose)
                .lineToY(-34)
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -44), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(57, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(66    , -10), Math.toRadians(0))
                .turnTo(Math.toRadians(270))
                .lineToY(-58)
                .splineToConstantHeading(new Vector2d(47, -58), Math.toRadians(270))
                .waitSeconds(1)
                .lineToY(-44)
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(270))
                .turnTo(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(270));




        waitForStart();

        while (opModeIsActive()) {

            gamepads.copyStates();
            if(gamepads.isPressed(-1, "cross")){
            trajectoryAction = testTraj.build();
            Actions.runBlocking(new SequentialAction(trajectoryAction));}


            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}


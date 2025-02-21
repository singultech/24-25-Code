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
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@Autonomous(name = "Pathing Tests", group = "Testing")
public class TestPathing extends LinearOpMode {
    @Override
    public void runOpMode() {
        Action trajectoryAction;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Grabber frontGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        VertSlidePair vertSlides = new VertSlidePair(4100, 1, hardwareMap);
        FrontArm arm = new FrontArm(hardwareMap);
        AutoSubsystems robot = new AutoSubsystems(arm, vertSlides, frontGrabber);
        Action Spec1 = drive.actionBuilder(initialPose)
                .lineToY(-34)
                .build();
        Action Samples = drive.actionBuilder(new Pose2d(10, -34, Math.toRadians(90)))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -44), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(57, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47, -58), Math.toRadians(0))
                .turnTo(Math.toRadians(270))
                .build();
        Action Spec2p2 = drive.actionBuilder(new Pose2d(47, -58, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(10, -34))
                .build();
        Action Spec3p1 = drive.actionBuilder(new Pose2d(10, -34, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(47, -58))
                .build();
        Action Spec3p2 = drive.actionBuilder(new Pose2d(47, -58, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(10, -34))
                .build();
        Action Spec4p1 = drive.actionBuilder(new Pose2d(10, -34, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(47, -58))
                .build();
        Action Spec4p2 = drive.actionBuilder(new Pose2d(47, -58, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(10, -34))
                .build();





        waitForStart();

        Actions.runBlocking(Spec1);
        Actions.runBlocking(Samples);
        Actions.runBlocking(Spec2p2);
        Actions.runBlocking(Spec3p1);
        Actions.runBlocking(Spec3p2);
        Actions.runBlocking(Spec4p1);
        Actions.runBlocking(Spec4p2);



    }
}


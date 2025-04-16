package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BackAssembly;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.HorizSlidePair;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@Config
@Autonomous(name = "SampleAuto",  preselectTeleOp="Teleop")
public class SampleAuto extends LinearOpMode {
    public static double x =34;
    public static double y =-43;
    public static double x1 =37;
    public static double y1 =-30;
    public static double s = 300;
    public static double roataions = -210;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(10, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        HorizSlidePair slides = new HorizSlidePair(hardwareMap);
        FrontArm frontArm = new FrontArm(hardwareMap);
        VertSlidePair vertslides = new VertSlidePair(hardwareMap);
        BackAssembly backAssembly = new BackAssembly(hardwareMap);
        Grabber frontGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));

        frontArm.setArmPosition(0.75);
        frontArm.setWristPosition(0.75);
        waitForStart();
        frontArm.setArmPosition(0.45);
        new Thread(() -> {
            while (!isStopRequested()) {
                slides.update();
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
                //backAssembly.update();
            }
        }).start();
        frontGrabber.close();
        new Thread(() -> {
            try { Thread.sleep(275); } catch (InterruptedException ignored) {}
            frontArm.setPosition(FrontArm.Position.HANG_PREP);
        }).start();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .waitSeconds(0.2)
                        .lineToY(-26)
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .waitSeconds(0.2)
                        .lineToY(-42)
                        .build());
        drive.updatePoseEstimate();

        frontGrabber.open();
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        new Thread(() -> {
            try {
                Thread.sleep(700);
            } catch (InterruptedException ignored) {

            }
            slides.setManualMode(false);
            slides.setTargetRotation(s);
        }).start();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(38, -30, Math.toRadians(240)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(37, -49, Math.toRadians(150)), Math.toRadians(240))
                        .build());
        drive.updatePoseEstimate();
        slides.forceStopPower();
        slides.setTargetRotation(-10);
        frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(new Pose2d(39, -16, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(60, -11), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(60, -50), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -56), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {

        }
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, -29), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        new Thread(() -> {
            try {
                Thread.sleep(1100);
            } catch (InterruptedException ignored) {

            }
            frontGrabber.open();
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) {

            }
            frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
            }).start();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(10, -45), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {

        }
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, -29), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        new Thread(() -> {
            try {
                Thread.sleep(1100);
            } catch (InterruptedException ignored) {

            }
            frontGrabber.open();

            frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        }).start();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(10, -45), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {

        }
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, -29), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        new Thread(() -> {
            try {
                Thread.sleep(1100);
            } catch (InterruptedException ignored) {

            }
            frontGrabber.open();
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) {

            }
            frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        }).start();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)

                        .splineToConstantHeading(new Vector2d(10, -45), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(37, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        }

    }
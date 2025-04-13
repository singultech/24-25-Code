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

@Config
@Autonomous(name = "SampleAuto")
public class SampleAuto extends LinearOpMode {
    public static double x =34;
    public static double y =-43;
    public static double x1 =37;
    public static double y1 =-30;
    public static double s = 375;
    public static double roataions = -210;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(10, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        HorizSlidePair slides = new HorizSlidePair(hardwareMap);
        BackAssembly backAssembly = new BackAssembly(hardwareMap);
        FrontArm frontArm = new FrontArm(hardwareMap);
        Grabber frontGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));

        frontArm.setPosition(FrontArm.Position.START);
        waitForStart();
        new Thread(() -> {
            while (!isStopRequested()) {
                slides.update();
                try { Thread.sleep(10); } catch (InterruptedException ignored) {}
                //backAssembly.update();
            }
        }).start();
        frontGrabber.close();
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .waitSeconds(0.2)
                        .lineToY(-26)
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToY(-42)
                        .build());
        drive.updatePoseEstimate();

        frontGrabber.open();
        slides.setManualMode(false);
        slides.setTargetRotation(s);
//        backAssembly.setTargetPreset(BackAssembly.Preset.FLOOR);
//        while (!slides.isAtTarget()){
//            telemetry.addData("slide pos", slides.getRotation());
//            telemetry.addData("slide target", slides.getTargetRotation());
//            telemetry.update();
//        }
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(280)), Math.toRadians(280))
                        .strafeTo(new Vector2d(x, y))
                        .build());
        drive.updatePoseEstimate();
        slides.forceStopPower();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(x1, y1))
                        .turn(Math.toRadians(roataions))
                        .build());
        drive.updatePoseEstimate();
        slides.setTargetRotation(0);
        frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(new Pose2d(39, -16, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(60, -11), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(60, -50), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(35.75, -56), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(35.75, -66), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(8, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(8, -31), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(8, -40), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.open();
        frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(35.5, -60))
                        .splineToConstantHeading(new Vector2d(35.5, -66), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(4, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(4, -31), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(4 , -40), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.open();
        frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(36, -60))
                        .splineToConstantHeading(new Vector2d(36, -66), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(2, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(2, -31), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(2, -40), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.open();
        }

    }
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
    public static double x = 34;
    public static double y = -43;
    public static double x1 = 37;
    public static double y1 = -30;
    public static double s = 1000;
    public static double roataions = -210;
    int a = -4;
    Pose2d beginPose;
    MecanumDrive drive;
    HorizSlidePair slides;
    FrontArm frontArm;
    VertSlidePair vertslides;
    BackAssembly backAssembly;
    Grabber frontGrabber;

    public void c(){
        if (frontGrabber.getSwitchState()) {
            hangSpec();
        }
        else{
            reGetSpec();
        }
    }

    public void reGetSpec() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ignored){}
        frontGrabber.open();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(39, -55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(39, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        c();
    }
    public void hangSpec() {
        frontGrabber.close();
        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {

        }
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(a, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(a, -29), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        new Thread(() -> {
            try {
                Thread.sleep(950);
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
                        .splineToConstantHeading(new Vector2d(39, -55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(39, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        a -= 2;
    }
    public void hangSpec2() {
        frontGrabber.close();
        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {

        }
        frontArm.setPosition(FrontArm.Position.HOPE_HANG);
        vertslides.setTargetPosition(VertSlidePair.SlidePosition.LIFT_FROM_WALL);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(a, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(a, -29), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        new Thread(() -> {
            try {
                Thread.sleep(100);
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
                        .splineToConstantHeading(new Vector2d(39, -55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(39, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        a -= 2;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        beginPose = new Pose2d(10, -62, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        slides = new HorizSlidePair(hardwareMap);
        frontArm = new FrontArm(hardwareMap);
        vertslides = new VertSlidePair(hardwareMap);
        backAssembly = new BackAssembly(hardwareMap);
        frontGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        new Thread(() -> {
            while (!isStopRequested()){
                telemetry.addData("Button state", frontGrabber.getSwitchState());
            }
        });
        frontArm.setArmPosition(0.75);
        frontArm.setWristPosition(0.75);
        waitForStart();
        frontArm.setArmPosition(0.45);
        frontGrabber.close();
        new Thread(() -> {
            try {
                Thread.sleep(275);
            } catch (InterruptedException ignored) {
            }
            frontArm.setPosition(FrontArm.Position.HANG_PREP);
        }).start();
        MecanumDrive.PARAMS.maxWheelVel = 60;
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .waitSeconds(0.2)
                        .lineToY(-26)
                        .build());
        MecanumDrive.PARAMS.maxWheelVel = 49;
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .waitSeconds(0.2)
                        .lineToY(-42)
                        .build());
        drive.updatePoseEstimate();
        new Thread(() -> {
            try {
                Thread.sleep(1);
            } catch (InterruptedException ignored) {

            }
//            slides.setManualMode(false);
//            slides.setTargetRotation((int) s);
        }).start();
        frontGrabber.open();
        frontArm.setPosition(FrontArm.Position.HANG_PREP);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(40, -30, Math.toRadians(240)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(40, -54, Math.toRadians(150)), Math.toRadians(240))
                        .build());
        drive.updatePoseEstimate();
        //slides.forceStopPower();
        new Thread(() -> {
//            slides.setTargetRotation((int) s+250);
            try {
                Thread.sleep(850);
            } catch (InterruptedException ignored) {

            }
//            slides.setTargetRotation(0);
        }).start();
        frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(new Pose2d(39, -16, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(57 , -11), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(57, -50), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(39, -56), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(39, -64), Math.toRadians(90))
                        .build());
        drive.updatePoseEstimate();
        frontGrabber.close();
        hangSpec();
        hangSpec();
        hangSpec();

        }
    }





//        frontGrabber.close();
//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException ignored) {
//
//        }
//        frontArm.setPosition(FrontArm.Position.HANG_PREP);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(0, -29), Math.toRadians(90))
//                        .build());
//        drive.updatePoseEstimate();
//        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
//        new Thread(() -> {
//            try {
//                Thread.sleep(1100);
//            } catch (InterruptedException ignored) {
//
//            }
//            frontGrabber.open();
//
//            frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
//        }).start();
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .splineToConstantHeading(new Vector2d(10, -45), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(39, -55), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(39, -64), Math.toRadians(90))
//                        .build());
//        drive.updatePoseEstimate();
//        frontGrabber.close();
//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException ignored) {
//
//        }
//        frontArm.setPosition(FrontArm.Position.HANG_PREP);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(0, -29), Math.toRadians(90))
//                        .build());
//        drive.updatePoseEstimate();
//        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
//        new Thread(() -> {
//            try {
//                Thread.sleep(1100);
//            } catch (InterruptedException ignored) {
//
//            }
//            frontGrabber.open();
//            try {
//                Thread.sleep(100);
//            } catch (InterruptedException ignored) {
//
//            }
//            frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
//        }).start();
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .splineToConstantHeading(new Vector2d(10, -45), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(39, -55), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(39, -64), Math.toRadians(90))
//                        .build());
//        drive.updatePoseEstimate();
//        frontGrabber.close();
//        }
//
//    }
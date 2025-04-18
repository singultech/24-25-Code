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
@Autonomous(name = "OthersAuto",  preselectTeleOp="Teleop")
public class OthersAuto extends LinearOpMode {
    public static double Dx =-49;
    public static double Dy =-46;
    public static double Dhs =350;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(-32.5, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        FrontArm frontArm = new FrontArm(hardwareMap);
        //HorizSlidePair slides = new HorizSlidePair(hardwareMap);
        //  slides.setManualMode(false);
        VertSlidePair vertslides = new VertSlidePair(hardwareMap);
        //BackAssembly backAssembly = new BackAssembly(hardwareMap);
        Grabber backGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("backGrabberServo"), hardwareMap.touchSensor.get("backGrabberSwitch"));
        Grabber frontGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));

        frontGrabber.close();

        waitForStart();
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(230)), Math.toRadians(270))
                .build());
        drive.updatePoseEstimate();
        vertslides.setTargetPosition(VertSlidePair.SlidePosition.MAX);
        while (Math.abs(vertslides.getTargetPosition(VertSlidePair.Side.LEFT)-vertslides.getPosition(VertSlidePair.Side.LEFT))>30){
            Thread.sleep(50);
        }
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Thread.sleep(2000);
        frontGrabber.open();
        frontArm.setPosition(FrontArm.Position.GRAB_FROM_WALL);
        vertslides.setTargetPosition(VertSlidePair.SlidePosition.ZERO);
        while (Math.abs(vertslides.getTargetPosition(VertSlidePair.Side.LEFT)-vertslides.getPosition(VertSlidePair.Side.LEFT))>30){
            Thread.sleep(50);
        }
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-30, -8, Math.toRadians(0)), Math.toRadians(270))
                .build());
        drive.updatePoseEstimate();
        vertslides.setTargetPosition(VertSlidePair.SlidePosition.LIFT_FROM_WALL);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-15, -8), Math.toRadians(270))
                .build());
        drive.updatePoseEstimate();
        Thread.sleep(2000);




//        backGrabber.open();
//        slides.setTargetRotation(Dhs);
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .waitSeconds(3)
//                        .build());
//        backAssembly.setTargetPreset(BackAssembly.Preset.FLOOR);
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .setTangent(Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(Dx, Dy), Math.toRadians(270))
//                        .build());
//        while (!backAssembly.atTarget()){
//            Thread.sleep(50);
//        }
//        Thread.sleep(500);
//        backGrabber.close();
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .waitSeconds(8)
//                        .build());
//        backAssembly.setTargetPreset(BackAssembly.Preset.FOLDED);
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .waitSeconds(8)


        drive.updatePoseEstimate();



    }

}
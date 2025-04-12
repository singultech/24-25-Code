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
import org.firstinspires.ftc.teamcode.subsystems.BackAssembly;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.HorizSlidePair;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(10, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        HorizSlidePair slides = new HorizSlidePair(hardwareMap);
        BackAssembly backAssembly = new BackAssembly(hardwareMap);
        FrontArm frontArm = new FrontArm(hardwareMap);
        Grabber frontGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));

        frontArm.setWristPosition(0.75);
        waitForStart();
        frontGrabber.close();
        frontArm.setPosition(FrontArm.Position.HANG_PREP);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .waitSeconds(0.5)
                        .lineToY(-28)
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .waitSeconds(0.5)
                        .lineToY(-43)
                        .build());
        drive.updatePoseEstimate();

        frontGrabber.open();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(240)), Math.toRadians(240))
                        .strafeTo(new Vector2d(37.5, -40.5))
                        .build());
        drive.updatePoseEstimate();
        slides.setManualMode(false);
        slides.setTargetRotation(400);
        slides.setManualMode(false);
        backAssembly.setTargetPreset(BackAssembly.Preset.FLOOR);
        while ((Math.abs(slides.getTargetRotation()-slides.getRotation())>30) || (Math.abs(backAssembly.getTargetPreset().getArmDegrees()-backAssembly.getCurrentPreset().getArmDegrees())>30)){
            slides.update();
            backAssembly.update();
            telemetry.addData("slides", slides.log());
            telemetry.addData("diffy", backAssembly);
            telemetry.update();
        }

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(38, -39))
                        .turnTo(Math.toRadians(140))
                        .turnTo(Math.toRadians(240))
                        .strafeTo(new Vector2d(43, -38))
                        .turnTo(Math.toRadians(140))
                        .turnTo(Math.toRadians(240))
                        .strafeTo(new Vector2d(50, -38))
                        .turnTo(Math.toRadians(140))
                        .build());
        drive.updatePoseEstimate();

    }
}
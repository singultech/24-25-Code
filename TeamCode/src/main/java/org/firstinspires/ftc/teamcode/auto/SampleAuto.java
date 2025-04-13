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
    public static double s = 388;
    public static double roataions = -190;

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
                        .lineToY(-28)
                        .build());
        drive.updatePoseEstimate();
        frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .waitSeconds(0.25)
                        .lineToY(-41)
                        .build());
        drive.updatePoseEstimate();

        frontGrabber.open();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(280)), Math.toRadians(280))
                        .strafeTo(new Vector2d(x, y))
                        .build());
        drive.updatePoseEstimate();
        slides.setManualMode(false);
        slides.setTargetRotation(s);
//        backAssembly.setTargetPreset(BackAssembly.Preset.FLOOR);
        while (!slides.isAtTarget()){
            telemetry.addData("slide pos", slides.getRotation());
            telemetry.addData("slide target", slides.getTargetRotation());
            telemetry.update();
        }
        slides.forceStopPower();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(x1, y1))
                        .turn(Math.toRadians(roataions))
                        /*.strafeTo(new Vector2d(43, -38))
                        .turnTo(Math.toRadians(140))
                        .turnTo(Math.toRadians(240))
                        .strafeTo(new Vector2d(50, -38))
                        .turnTo(Math.toRadians(140))

                         */
                        .build());
        drive.updatePoseEstimate();

        backAssembly.setTargetPreset(BackAssembly.Preset.FOLDED);
        while (!backAssembly.atTarget()){
            backAssembly.update();
            telemetry.addData("diffy", backAssembly);
            telemetry.update();
        }

    }
}
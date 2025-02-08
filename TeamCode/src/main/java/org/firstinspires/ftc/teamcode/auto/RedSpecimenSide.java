package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.BackArm;
import org.firstinspires.ftc.teamcode.utils.FrontArm;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.Grabber;
import org.firstinspires.ftc.teamcode.utils.VertSlidePair;

@Autonomous(name = "Red - Specimen Side", group = "aRed", preselectTeleOp = "Teleop")
public class RedSpecimenSide extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(12, -70, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        VertSlidePair vertSlides = new VertSlidePair(4100, hardwareMap);
        Grabber frontGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        FrontArm arm = new FrontArm(1, 0.7, hardwareMap);
        waitForStart();
        frontGrabber.close();
        arm.forward();
        Action goToSub = drive.actionBuilder(drive.pose)
                .lineToY(-45)
                .build();
        Actions.runBlocking(goToSub);
        vertSlides.changeTargetPosition(3500);
        sleep(1500);
        Action goForward = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .lineToY(-38)
                .build();
        Actions.runBlocking(goForward);
        vertSlides.changeTargetPosition(-800);
        sleep(700);
        frontGrabber.open();
        Action goBack = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .lineToY(-45)
                .build();
        Actions.runBlocking(goBack);
        vertSlides.setTargetPosition(0);
        Action shiftRight = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(41,-45), Math.toRadians(90))
                .build();
        Actions.runBlocking(shiftRight);
        Action shiftForward = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(41,-20), Math.toRadians(90))
                .turnTo(Math.toRadians(90))
                .build();
        Actions.runBlocking(shiftForward);
        Action shiftFinalRight = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(56,-20))
                .turnTo(Math.toRadians(90))
                .build();
        Actions.runBlocking(shiftFinalRight);
        Action depo = drive.actionBuilder(drive.pose)
                .lineToY(-70)
                .turnTo(Math.toRadians(90))
                .build();
        Actions.runBlocking(depo);
        Action shiftFAgain = drive.actionBuilder(drive.pose)
                .lineToY(-20)
                .turnTo(Math.toRadians(90))
                .build();
        Actions.runBlocking(shiftFAgain);
        Action shiftRAgain = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(65,-20))
                .turnTo(Math.toRadians(90))
                .build();
        Actions.runBlocking(shiftRAgain);
        Action shiftBAgain = drive.actionBuilder(drive.pose)
                .lineToY(-70)
                .build();
        Actions.runBlocking(shiftBAgain);
        Action shiftFAAgain = drive.actionBuilder(drive.pose)
                .lineToY(-20)
                .build();
        Actions.runBlocking(shiftFAAgain);
        Action shiftRAAgain = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(70,-20))
                .turnTo(Math.toRadians(90))
                .build();
        Actions.runBlocking(shiftRAAgain);
        Action park = drive.actionBuilder(drive.pose)
                .lineToY(-70)
                .build();
        Actions.runBlocking(park);
        vertSlides.setTargetPosition(0);





        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }
}


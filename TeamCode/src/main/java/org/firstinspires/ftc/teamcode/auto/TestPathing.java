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
import org.firstinspires.ftc.teamcode.utils.GamepadPair;

@Autonomous(name = "Pathing Tests", group = "Testing")
public class TestPathing extends LinearOpMode {
    @Override
    public void runOpMode() {
        Action trajectoryActionChosen;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        TrajectoryActionBuilder forwardTrajectory = drive.actionBuilder(initialPose)
                .lineToY(20)
                .waitSeconds(2)
                .lineToY(15);
        TrajectoryActionBuilder sidewaysTrajectory = drive.actionBuilder(initialPose)
                .lineToX(20)
                .waitSeconds(2)
                .lineToX(15);
        TrajectoryActionBuilder turnTrajectory = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(90));
        TrajectoryActionBuilder cookedTrajectory = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(15, 15), Math.toRadians(0));


        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            if(gamepads.isPressed(-1, "circle")){
                trajectoryActionChosen = forwardTrajectory.build();
                Actions.runBlocking(new SequentialAction(trajectoryActionChosen));
            }
            if(gamepads.isPressed(-1, "cross")){
                trajectoryActionChosen = sidewaysTrajectory.build();
                Actions.runBlocking(new SequentialAction(trajectoryActionChosen));
            }
            if(gamepads.isPressed(-1, "square")){
                trajectoryActionChosen = turnTrajectory.build();
                Actions.runBlocking(new SequentialAction(trajectoryActionChosen));
            }
            if(gamepads.isPressed(-1, "triangle")){
                trajectoryActionChosen = cookedTrajectory.build();
                Actions.runBlocking(new SequentialAction(trajectoryActionChosen));
            }


            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}


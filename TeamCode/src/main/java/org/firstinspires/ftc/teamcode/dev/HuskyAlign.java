package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Huskylens;

@Autonomous(name = "Husky Align", group = "dev")
public class HuskyAlign extends LinearOpMode {
    private MecanumDrive drive;

    public static int TOLERANCE_FROM_CENTER = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose2d(10, -60, Math.toRadians(0)));
        Grabber grabber = new Grabber(0.73, 1,
                hardwareMap.servo.get("frontGrabberServo"),
                hardwareMap.touchSensor.get("frontGrabberSwitch"));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Huskylens husky = new Huskylens(hardwareMap);
        String side;
        grabber.open();
        waitForStart();


        while (!grabber.getSwitchState() && !isStopRequested()) {
            HuskyLens.Block sample = husky.getBiggestBlock();
            if(sample != null) {
                if (Math.abs(sample.x - 160) < TOLERANCE_FROM_CENTER) side = "CENTER";
                else if (sample.x < 160) side = "LEFT";
                else side = "RIGHT";

                telemetry.addData("Sample", sample);
                telemetry.addData("Side sample is on", side);
            }
            drive.updatePoseEstimate();
            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
        telemetry.clear();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(0));
        Actions.runBlocking(tab1.build());
        while (!isStopRequested()){
            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }

    private void runAction(Action action) {
        Actions.runBlocking(action);
    }
}
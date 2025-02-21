package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
    private MecanumDrive drive;
    private Grabber grabber;
    private VertSlidePair slides;
    private FrontArm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Robot initialization
        Grabber grabber = new Grabber(0.73, 1,
                hardwareMap.servo.get("frontGrabberServo"),
                hardwareMap.touchSensor.get("frontGrabberSwitch"));
        VertSlidePair slides = new VertSlidePair(4100, 1, hardwareMap);
        FrontArm arm = new FrontArm(hardwareMap);
        initializeRobot();

        waitForStart();

        grabber.close();
        arm.setPosition(1);
        slides.changeTargetPosition("l", 2000);
        runAction(drive.actionBuilder(drive.pose)
                .lineToY(-44)
                .build());
        slides.setTargetPosition("l", 3700);
        sleep(950);
        grabber.open();
        arm.setPosition(0);
        slides.setTargetPosition("l", 0);


        // Sample movements
        runAction(drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -44), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(49, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(49, -52))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(60, -60))
                .splineToConstantHeading(new Vector2d(48.25, -58), Math.toRadians(270))
                .turnTo(Math.toRadians(90))
                .lineToY(-63.25, new TranslationalVelConstraint(20.0))
                .build());
        grabber.close();
        sleep(500);
        /*
        // Run repeated movement patterns
        for (int i = 0; i < 3; i++) {
            runAction(drive.actionBuilder(drive.pose)
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(10, -34))
                    .build());

            // Move to second position
            runAction(drive.actionBuilder(drive.pose)
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(47, -58))
                    .build());
        }
         */
    }

    private void initializeRobot() {
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize subsystems
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
    }

    private void runAction(Action action) {
        Actions.runBlocking(action);
    }
}
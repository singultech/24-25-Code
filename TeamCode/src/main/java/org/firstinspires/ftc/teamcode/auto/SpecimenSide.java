package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.HorizSlidePair;
import org.firstinspires.ftc.teamcode.subsystems.Huskylens;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@Autonomous(name = "Auto - Specimen Side", group = "Testing")
public class SpecimenSide extends LinearOpMode {
    private MecanumDrive drive;
    private Grabber grabber;
    private VertSlidePair slides;
    private FrontArm arm;
    private static final double CENTER_X = 160.0;
    private static final double MAX_SPEED = 0.4; // Maximum speed
    private static final double MIN_SPEED = 0.2; // Minimum speed to maintain movement
    private static final double DEAD_ZONE = 16; // Dead zone around center
    private static final double FORWARD_SPEED = 0.3; // Speed for forward movement when centered
    boolean isCentered = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Robot initialization
        Grabber grabber = new Grabber(0.73, 1,
                hardwareMap.servo.get("frontGrabberServo"),
                hardwareMap.touchSensor.get("frontGrabberSwitch"));
        VertSlidePair slides = new VertSlidePair(4100, 1, hardwareMap);
        Huskylens husky = new Huskylens(hardwareMap);
        BackArm backArm = new BackArm(hardwareMap, true);
        FrontArm arm = new FrontArm(hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        backArm.setTargetRotation(-530);
        while (Math.abs(backArm.getPosition()-backArm.getTargetRotation())>10 && !isStopRequested()){
            backArm.update();
        }

        waitForStart();
        Pose2d initialPose = new Pose2d(10, -62.75, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);


        grabber.close();
        arm.setPosition(1);
        slides.changeTargetPosition(2810);
        backArm.setTargetRotation(0);
        while (Math.abs(backArm.getPosition()-backArm.getTargetRotation())>10){
            backArm.update();
        }
        runAction(drive.actionBuilder(drive.pose)
                .lineToY(-43.1, new TranslationalVelConstraint(30.0))
                .build());
        slides.setTargetPosition(3700);
        sleep(950);
        grabber.open();
        arm.setPosition(0);
        slides.setTargetPosition(0);


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
                .lineToY(-57)
                .build());
        while (!grabber.getSwitchState() && !isStopRequested()) {
            String direction = "";
            double strafeSpeed = 0;
            double forwardSpeed = 0;

            for(HuskyLens.Block block : husky.getBlocks()) {
                double distanceFromCenter = block.x - CENTER_X;

                if (Math.abs(distanceFromCenter) < DEAD_ZONE) {
                    direction = "Center";
                    strafeSpeed = 0;
                    forwardSpeed = -FORWARD_SPEED; // Move forward once centered
                } else {
                    // Only align if not yet centered
                    direction = distanceFromCenter < 0 ? "Left" : "Right";
                    strafeSpeed = Math.abs(distanceFromCenter) / CENTER_X * MAX_SPEED;
                    strafeSpeed = Math.max(strafeSpeed, MIN_SPEED);
                    strafeSpeed = Math.min(strafeSpeed, MAX_SPEED);

                    if (direction.equals("Left")) {
                        strafeSpeed = -strafeSpeed;
                    }
                    forwardSpeed = 0;
                }

                telemetry.addLine(block.id + " X:" + block.x + " Y:" + block.y + " Wi:" + block.width + " He:" + block.height);
                telemetry.addLine("Move: " + direction);
                telemetry.addLine("Strafe Speed: " + String.format("%.3f", strafeSpeed));
                telemetry.addLine("Forward Speed: " + String.format("%.3f", forwardSpeed));
                telemetry.addData("button:", grabber.getSwitchState());
            }
            if (husky.getBlocks().length == 0){
                forwardSpeed = -FORWARD_SPEED;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(forwardSpeed, strafeSpeed),
                    0
            ));
            drive.updatePoseEstimate();

            telemetry.update();
        }

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0, 0),
                0
        ));
        drive.updatePoseEstimate();
        grabber.close();
        slides.changeTargetPosition(200);
        sleep(50);
        // Run repeated movement patterns
        for (int i = 0; i < 2; i++) {
            while (!grabber.getSwitchState() && !isStopRequested()) {
                String direction = "";
                double strafeSpeed = 0;
                double forwardSpeed = 0;

                for(HuskyLens.Block block : husky.getBlocks()) {
                    double distanceFromCenter = block.x - CENTER_X;

                    if (Math.abs(distanceFromCenter) < DEAD_ZONE) {
                        direction = "Center";
                        strafeSpeed = 0;
                        forwardSpeed = -FORWARD_SPEED; // Move forward once centered
                    } else {
                        // Only align if not yet centered
                        direction = distanceFromCenter < 0 ? "Left" : "Right";
                        strafeSpeed = Math.abs(distanceFromCenter) / CENTER_X * MAX_SPEED;
                        strafeSpeed = Math.max(strafeSpeed, MIN_SPEED);
                        strafeSpeed = Math.min(strafeSpeed, MAX_SPEED);

                        if (direction.equals("Left")) {
                            strafeSpeed = -strafeSpeed;
                        }
                        forwardSpeed = 0;
                    }

                    telemetry.addLine(block.id + " X:" + block.x + " Y:" + block.y + " Wi:" + block.width + " He:" + block.height);
                    telemetry.addLine("Move: " + direction);
                    telemetry.addLine("Strafe Speed: " + String.format("%.3f", strafeSpeed));
                    telemetry.addLine("Forward Speed: " + String.format("%.3f", forwardSpeed));
                    telemetry.addData("button:", grabber.getSwitchState());
                }
                if (husky.getBlocks().length == 0){
                    forwardSpeed = -FORWARD_SPEED;
                }
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(forwardSpeed, strafeSpeed),
                        0
                ));
                drive.updatePoseEstimate();

                telemetry.update();
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, 0),
                    0
            ));
            drive.updatePoseEstimate();
            grabber.close();
            slides.changeTargetPosition(200);
            sleep(50);
            arm.forward();
            slides.setTargetPosition(2810);
            runAction(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(15, -48))
                    .lineToY(-43.65)
                    .build());
            drive.updatePoseEstimate();
            slides.setTargetPosition(3700);
            sleep(950);
            grabber.open();
            arm.setPosition(0);
            slides.setTargetPosition(0);

            // Move to second position
            runAction(drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(47, -58))
                    .build());
        }
    }

    private void runAction(Action action) {
        Actions.runBlocking(action);
    }
}
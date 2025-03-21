package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Huskylens;

@Autonomous(name = "Husky Align", group = "Testing")
public class HuskyAlign extends LinearOpMode {
    private MecanumDrive drive;
    private static final double CENTER_X = 160.0;
    private static final double MAX_SPEED = 0.3; // Maximum speed
    private static final double MIN_SPEED = 0.22; // Minimum speed to maintain movement
    private static final double DEAD_ZONE = 15; // Dead zone around center
    private static final double FORWARD_SPEED = 0.3; // Speed for forward movement when centered

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        Grabber grabber = new Grabber(0.73, 1,
                hardwareMap.servo.get("frontGrabberServo"),
                hardwareMap.touchSensor.get("frontGrabberSwitch"));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Huskylens husky = new Huskylens(hardwareMap);
        grabber.open();
        waitForStart();

        boolean isCentered = false;

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

            telemetry.update();
        }

        // Stop the robot once the switch is pressed
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0, 0),
                0
        ));
        grabber.close();
    }

    private void runAction(Action action) {
        Actions.runBlocking(action);
    }
}
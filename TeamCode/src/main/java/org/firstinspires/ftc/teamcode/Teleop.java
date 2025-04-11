package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.HorizSlidePair;
import org.firstinspires.ftc.teamcode.subsystems.RumbleEffects;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@TeleOp(name = "*Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // region definitions
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Grabber frontGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        Grabber backGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("backGrabberServo"), hardwareMap.touchSensor.get("backGrabberSwitch"));
        VertSlidePair vertSlides = new VertSlidePair(hardwareMap);
        HorizSlidePair horizSlides = new HorizSlidePair(hardwareMap);
        BackArm backArm = new BackArm(hardwareMap);
        FrontArm frontArm = new FrontArm(hardwareMap);
        Diffy diffy = new Diffy(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //endregion

        //region variables
        String driveStyle = "robot-centric";
        long curTime;
        long lastFrontOpened = 0;
        long lastBackOpened = 0;
        //endregion

        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            horizSlides.update();
            curTime = System.currentTimeMillis();

            //region drivecode
            double driveScaleFactor = 1-gamepads.getTrigger(1, "right_trigger");
            Vector2d driveVector;
            float gpx = -gamepads.joystickValue(1, "left", "y");
            float gpy = -gamepads.joystickValue(1, "left", "x");
            if (driveStyle.equals("field-centric")) {
                float head = (float) -drive.pose.heading.toDouble();
                driveVector = new Vector2d(
                        ((gpx * Math.cos(head)) - (gpy * Math.sin(head))) * driveScaleFactor,
                        ((gpx * Math.sin(head)) + (gpy * Math.cos(head))) * driveScaleFactor
                );
            } else {
                driveVector = new Vector2d(gpx * driveScaleFactor, gpy * driveScaleFactor);
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    driveVector,
                    -(gamepads.joystickValue(1, "right", "x")*driveScaleFactor)
            ));
            //endregion

            //region Switch Drive Style
            if (gamepads.isPressed(1, "left_stick_button")){
                if (driveStyle.equals("field-centric")) driveStyle = "robot-centric";
                else driveStyle = "field-centric";
            }
            //endregion

            //region Snap to 90 degree
            if (gamepads.isPressed(1, "right_stick_button")){
                double currentHeading = drive.pose.heading.toDouble();
                double degrees = Math.toDegrees(currentHeading) % 360;
                if (degrees < 0) degrees += 360;
                double targetDegrees = Math.round(degrees / 90.0) * 90.0;
                Action snapTo90 = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(targetDegrees))
                        .build();
                Actions.runBlocking(snapTo90);
            }
            //endregion

            //region Vert Slide Control
            if (gamepads.isPressed(1, "dpad_up")) vertSlides.incrementSlidePreset(1);
            if (gamepads.isPressed(1, "dpad_down")) vertSlides.incrementSlidePreset(-1);

            //endregion


            //region Diffy control
            double verticalPower = gamepads.joystickValue(2, "left", "y");
            double rotationPower = -gamepads.joystickValue(2, "right", "x");

            double leftPower = (verticalPower + rotationPower) *0.5;
            double rightPower = (verticalPower - rotationPower) *0.5;

            diffy.setLeftPower(leftPower);
            diffy.setRightPower(rightPower);


            if (gamepads.isHeld(2, "dpad_right")) {
                backArm.setManualPower(1);
            } else if (gamepads.isHeld(2, "dpad_left")) backArm.setManualPower(-1);
            else backArm.setManualPower(0);
            //endregion

            //region Horiz Slide Control
            if (gamepads.isPressed(1, "square")){
                if (horizSlides.getTargetRotation()==0){
                    horizSlides.setTargetRotation(350);
                } else {
                    horizSlides.setTargetRotation(0);
                }
            }
            //endregion

            //region Toggle Grabber Code
            // Toggle front grabber
            if (gamepads.isPressed(1, "circle")) {
                if (frontGrabber.isClosed()) {
                    frontGrabber.open();
                    lastFrontOpened = curTime;
                }
                else {
                    frontGrabber.close();
                }
            }
            // Toggle back grabber
            if (gamepads.isPressed(2, "circle")) {
                if (backGrabber.isClosed()) {
                    backGrabber.open();
                    lastBackOpened = curTime;
                }
                else {
                    backGrabber.close();
                }
            }
            //endregion

            //region Close Grabbers on button press
            if (frontGrabber.getSwitchState() && curTime - lastFrontOpened > 2000 && !frontGrabber.isClosed()){
                new Thread(() -> {
                    try {
                        frontGrabber.close();
                        gamepads.blipRumble(-1, 1);

                        Thread.sleep(750);

                        if (!frontGrabber.getSwitchState()) {frontGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }
            if (backGrabber.getSwitchState() && curTime - lastBackOpened > 2000 && !backGrabber.isClosed()){
                new Thread(() -> {
                    try {
                        backGrabber.close();
                        gamepads.blipRumble(1, 1);

                        Thread.sleep(750);

                        if (!backGrabber.getSwitchState()) {backGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }).start();
            }
            //endregion


            drive.updatePoseEstimate();

            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Drive style", driveStyle);
            telemetry.update();

        }
    }
}
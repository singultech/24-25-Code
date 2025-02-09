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

import org.firstinspires.ftc.teamcode.utils.BackArm;
import org.firstinspires.ftc.teamcode.utils.Diffy;
import org.firstinspires.ftc.teamcode.utils.FrontArm;
import org.firstinspires.ftc.teamcode.utils.GamepadPair;
import org.firstinspires.ftc.teamcode.utils.Grabber;
import org.firstinspires.ftc.teamcode.utils.HorizSlidePair;
import org.firstinspires.ftc.teamcode.utils.RumbleEffects;
import org.firstinspires.ftc.teamcode.utils.VertSlidePair;

@TeleOp(name = "*Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // region definitions
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Grabber frontGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        Grabber backGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("backGrabberServo"), hardwareMap.touchSensor.get("backGrabberSwitch"));
        VertSlidePair vertSlides = new VertSlidePair(4100, 1, hardwareMap);
        HorizSlidePair horizSlides = new HorizSlidePair(hardwareMap, true);
        BackArm backArm = new BackArm(0.55, 0, hardwareMap, true);
        FrontArm frontArm = new FrontArm(0.98, 0.35, hardwareMap);
        Diffy diffy = new Diffy(hardwareMap, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //endregion

        //region variables
        String driveStyle = "robot-centric";
        long curTime;
        long lastFrontOpened = 0;
        long lastBackOpened = 0;
        int grabOffWall = 850;
        int aboveTopBar = 3550;
        int hangHeight = 2050;
        int[] vertSlidePresets = {0, grabOffWall, aboveTopBar, hangHeight};
        frontArm.forward();
        int vertSlidePreset = 0;
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

            //region Slide preset control
            if (gamepads.isPressed(1, "dpad_up") && vertSlidePreset+1 < vertSlidePresets.length) {
                vertSlidePreset++;
                vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);
            }
            if (gamepads.isPressed(1, "dpad_down") && vertSlidePreset>0) {
                vertSlidePreset--;
                vertSlides.setTargetPosition(vertSlidePresets[vertSlidePreset]);
            }
            //endregion

            //region Back Arm Control
            if (gamepads.isHeld(-1, "dpad_right")) {
                backArm.setPower(1);
            } else if (gamepads.isHeld(-1, "dpad_left")) backArm.setPower(-1);
            else backArm.setPower(0);
            //endregion

            //region Horiz Slide Control
            if (gamepads.isPressed(1, "square")){
                if (horizSlides.getTargetRotation()==0){
                    horizSlides.setTargetRotation(400);
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
                int currentVertSlidePreset = vertSlidePreset;
                new Thread(() -> {
                    try {
                        frontGrabber.close();
                        gamepads.blipRumble(-1, 1);

                        Thread.sleep(750);

                        if (!frontGrabber.getSwitchState()) {frontGrabber.open(); gamepads.rumble(1, RumbleEffects.alternating);}
                        if (frontGrabber.getSwitchState() && vertSlidePresets[currentVertSlidePreset] == grabOffWall){
                            vertSlides.changeTargetPosition(300);
                            gamepads.blipRumble(1,  1);
                        }

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

            //region Lower and release to bar on bumper press
            if (vertSlidePresets[vertSlidePreset] == aboveTopBar && gamepads.isPressed(1, "right_bumper")){
                new Thread(() -> {
                    vertSlides.changeTargetPosition(-1000);
                    while (Math.abs(vertSlides.getCurrentPosition() - vertSlides.getTargetPosition()) > 15) {
                        try {
                            Thread.sleep(10);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                    frontGrabber.open();
                    gamepads.blipRumble(1, 1);
                }).start();
            }
            //endregion

            //region Pick up from wall on bumper press
            if (vertSlidePresets[vertSlidePreset] == grabOffWall && gamepads.isPressed(1, "right_bumper")){
                frontGrabber.close();
                vertSlides.changeTargetPosition(300);
                gamepads.blipRumble(1,  1);
            }
            //endregion

            drive.updatePoseEstimate();

            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Slide preset", vertSlidePreset);
            telemetry.addData("Drive style", driveStyle);
            telemetry.update();

        }
    }
}
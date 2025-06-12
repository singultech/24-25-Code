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
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.subsystems.BackAssembly;
import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.HorizSlidePair;
import org.firstinspires.ftc.teamcode.subsystems.RumbleEffects;
import org.firstinspires.ftc.teamcode.subsystems.VertSlidePair;

@TeleOp(name = "*Teleop")
public class Teleop extends LinearOpMode {

    public static double diffykP = 0.01;
    public static double diffykI = 0.0;
    public static double diffykD = 0.0;
    public static double armkP = 0.012;
    public static double armkI = 0.02;
    public static double armkD = 0.0005;
    @Override
    public void runOpMode() throws InterruptedException {

        // region definitions
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Grabber frontGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
        Grabber backGrabber = new Grabber(0.70, 1, hardwareMap.servo.get("backGrabberServo"), hardwareMap.touchSensor.get("backGrabberSwitch"));
        VertSlidePair vertSlides = new VertSlidePair(hardwareMap);
        HorizSlidePair horizSlides = new HorizSlidePair(hardwareMap);
        BackAssembly backAssembly = new BackAssembly(hardwareMap);
        FrontArm frontArm = new FrontArm(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive.PARAMS.maxWheelVel = 80;
        //endregion

        //region variables
        String driveStyle = "robot-centric";
        long curTime;
        long lastFrontOpened = 0;
        long lastBackOpened = 0;
        boolean backAtTarget = true;
        boolean horizSlidesAtTarget = true;
        //endregion

        waitForStart();
        vertSlides.setHook(Hook.HookPosition.UP);

        while (opModeIsActive()) {
            gamepads.copyStates();
//            backAssembly.update();
//            horizSlides.update();

            if (!gamepads.isHeld(2, "right_bumper")) {
                backAssembly.update();
            }
//            if(!horizSlidesAtTarget) {
//                horizSlides.update();
//                if(horizSlides.isAtTarget()) horizSlidesAtTarget = true;
//            }
            curTime = System.currentTimeMillis();
            backAssembly.getDiffy().setPidCoeffs(diffykP, diffykI, diffykD);
            backAssembly.getBackArm().setPidCoeffs(armkP, armkI, armkD);;

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

            //region Front arm control
            if (gamepads.isPressed(1,"dpad_right")) frontArm.incrementPreset(1);
            if (gamepads.isPressed(1, "dpad_left")) frontArm.incrementPreset(-1);


            //region Backassembly control
            if (gamepads.isPressed(1, "triangle")){
                if(backAssembly.getTargetPreset() == BackAssembly.Preset.FOLDED){
                    backAssembly.setTargetPreset(BackAssembly.Preset.RIGHT_ABOVE_FLOOR);
                    backAtTarget = false;
                }
                else if (backAssembly.getTargetPreset() == BackAssembly.Preset.RIGHT_ABOVE_FLOOR){
                    backAssembly.setTargetPreset(BackAssembly.Preset.FLOOR);
                    backAtTarget = false;
                }
                else {
                    backAssembly.setTargetPreset(BackAssembly.Preset.FOLDED);
                    backAtTarget = false;
                }
            }
            if(gamepads.isPressed("right_bumper")) {
                backAssembly.getDiffy().rollGrabber(10);
                backAtTarget = false;
            }
            if(gamepads.isPressed("left_bumper")) {
                backAssembly.getDiffy().rollGrabber(-10);
                backAtTarget = false;
            }
            //endregion

            //region Horiz Slide Control
            if (gamepads.isPressed(1, "square")){
                if (horizSlides.getTargetRotation()==250){
                    horizSlides.setTargetRotation(5);
                    horizSlidesAtTarget = false;
                } else {
                    horizSlides.setTargetRotation(250);
                    horizSlidesAtTarget = false;
                }
            }
            //endregion

            //region Toggle Grabber Code
            if (gamepads.isPressed( "circle")) {
                if (frontGrabber.isClosed()) {
                    frontGrabber.open();
                    lastFrontOpened = curTime;
                }
                else {
                    frontGrabber.close();
                }
                if (backGrabber.isClosed()) {
                    backGrabber.open();
                    lastBackOpened = curTime;
                }
                else {
                    backGrabber.close();
                }
            }
            //endregion

            //region Close Grabbers on switch press
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

            if(gamepads.isHeld(2, "dpad_right")){
                horizSlides.setManualMode(true);
                horizSlides.setManualPower(1);
            }
            if(gamepads.isHeld(2, "dpad_left")){
                horizSlides.setManualMode(true);
                horizSlides.setManualPower(-1);
            }
            //endregion


            drive.updatePoseEstimate();

            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Drive style", driveStyle);
            telemetry.addData("Horiz Slides target ", horizSlides.getTargetRotation());
            telemetry.addData("Back Preset: ", backAssembly.getTargetPreset());
            telemetry.addData("Left Diffy Angle", ((hardwareMap.get(AnalogInput.class, "leftDiffyEncoder").getVoltage()/3.3)*360));

            telemetry.update();

        }
    }
}
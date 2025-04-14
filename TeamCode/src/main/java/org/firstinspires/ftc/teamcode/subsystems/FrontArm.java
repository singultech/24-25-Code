package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class FrontArm {
    private final Servo armServo;
    private final Servo wristServo;
    private Position currentPosition;


    public enum Position {
        GRAB_FROM_WALL(0.75,0.5),
        HANG_PREP(0,0.35),
        HANG_SPECIMEN(0, 0.075);

        private final double armPosition;
        private final double wristPosition;
        Position(double armPosition, double wristPosition){
            this.armPosition = armPosition;
            this.wristPosition = wristPosition;
        }
        public double getArmPosition(){
            return armPosition;
        }
        public double getWristPosition(){
            return wristPosition;
        }
    }

    public FrontArm(HardwareMap hmap){
        armServo = hmap.servo.get("frontArmServo");
        wristServo = hmap.servo.get("wristServo");
        armServo.setDirection(Servo.Direction.REVERSE);
        setPosition(Position.GRAB_FROM_WALL);
    }

    public void setPosition(Position position){
        setArmPosition(position.getArmPosition());
        setWristPosition(position.getWristPosition());
        currentPosition = position;
    }

    public Position getPosition(){return currentPosition;}

    public void setArmPosition(double position){
        if(position >= 0 && position <= 1) {
            armServo.setPosition(position);
        }
    }
    public void setWristPosition(double position){
        if(position >= 0 && position <= 1) {
            wristServo.setPosition(position);
        }
    }

    private int getIndexOfPreset(Position preset){
        for(int i = 0; i < Position.values().length; i++){
            if(Position.values()[i] == preset) return i;
        }
        return -1;
    }
    public void incrementPreset(int amount){
        int currentPresetIndex = getIndexOfPreset(currentPosition);
        if (currentPresetIndex == -1) return;
        if (currentPresetIndex + amount < 0 || currentPresetIndex + amount >= Position.values().length) return;
        Position newPreset = Position.values()[currentPresetIndex + amount];
        setPosition(newPreset);
    }


    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Front Arm/Wrist Position: %s", getPosition());
    }

    @TeleOp(name = "FrontArmTest")
    public static class FrontArmTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FrontArm frontArm = new FrontArm(hardwareMap);
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
            Grabber frontGrabber = new Grabber(0.73, 1, hardwareMap.servo.get("frontGrabberServo"), hardwareMap.touchSensor.get("frontGrabberSwitch"));
            frontGrabber.close();
            waitForStart();
            while (!isStopRequested()) {
                gamepads.copyStates();

                double driveScaleFactor = 1-gamepads.getTrigger(1, "right_trigger");
                Vector2d driveVector;
                float gpx = -gamepads.joystickValue(1, "left", "y");
                float gpy = -gamepads.joystickValue(1, "left", "x");
                driveVector = new Vector2d(gpx * driveScaleFactor, gpy * driveScaleFactor);
                drive.setDrivePowers(new PoseVelocity2d(
                        driveVector,
                        -(gamepads.joystickValue(1, "right", "x")*driveScaleFactor)
                ));
                if(gamepads.isPressed("cross")){
                    if(frontGrabber.isClosed()) frontGrabber.open();
                    else frontGrabber.close();
                }
                if (gamepads.isPressed("up_dpad")) frontArm.setPosition(FrontArm.Position.HANG_PREP);
                if (gamepads.isPressed("right_dpad")) frontArm.setPosition(Position.HANG_SPECIMEN);
                if (gamepads.isPressed("down_dpad")) frontArm.setPosition(Position.GRAB_FROM_WALL);

                telemetry.addLine(frontArm.toString());
                telemetry.update();
            }
        }
    }


}

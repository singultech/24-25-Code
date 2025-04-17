package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class BackAssembly {
    private final BackArm backArm;
    private final Diffy diffy;

    private Preset currentPreset;
    private Preset targetPreset;

    public BackAssembly(HardwareMap hmap) {
        backArm = new BackArm(hmap);
        diffy = new Diffy(hmap);
        diffy.setMaxPower(0.15);
        currentPreset = Preset.FOLDED;
        targetPreset = Preset.FOLDED;
        backArm.setPidCoeffs(0.012,0.02,0.0005);
        diffy.setPidCoeffs(0.01, 0.02, 0.0005);
    }

    public enum Preset {
        FOLDED(0, 0),
        TRANSFER(45, 0),
        MIDWAY(93, 90),
        CHILL_GUY(160, 110),
        ABOVE_FLOOR(170, 190),
        EXTRA(188, 190),
        FLOOR(188, 280);

        final int armDegrees;
        final int diffyDegrees;

        Preset(int armDegrees, int diffyDegrees) {
            this.armDegrees = armDegrees;
            this.diffyDegrees = diffyDegrees;
        }

        public int getArmDegrees() {
            return armDegrees;
        }

        public int getDiffyDegrees() {
            return diffyDegrees;
        }
    }

    private int getIndexOfPreset(Preset preset){
        for(int i = 0; i < Preset.values().length; i++){
            if(Preset.values()[i] == preset) return i;
        }
        return -1;
    }

    public void update() {
        backArm.update();
        diffy.update();
        if(targetPreset == currentPreset) return;
        if (!backArm.isAtTarget(15) || !diffy.isAtTarget(17)) return;
        if(getIndexOfPreset(targetPreset) > getIndexOfPreset(currentPreset)){
            setPreset(Preset.values()[getIndexOfPreset(currentPreset) + 1]);
        } else {
            setPreset(Preset.values()[getIndexOfPreset(currentPreset) - 1]);
        }

    }

    public void setTargetPreset(Preset target) {
        this.targetPreset = target;
    }
    private void setPreset(Preset preset){
        currentPreset = preset;
        backArm.setTargetRotation(preset.getArmDegrees());
        diffy.setTargetRotation(preset.getDiffyDegrees());
    }

    public Preset getCurrentPreset() {
        return currentPreset;
    }
    public Preset getTargetPreset() {
        return targetPreset;
    }
    public Diffy getDiffy(){
        return diffy;
    }
    public BackArm getBackArm(){
        return backArm;
    }

    public boolean atTarget(){
        return backArm.isAtTarget() && diffy.isAtTarget();
    }

    @NonNull
    public String toString(){
        return diffy.log() + "\n" + backArm;
    }

    @Config
    @TeleOp(name = "Back Assembly")
    public static class BackAssemblyOpmode extends LinearOpMode {
        public static double diffykP = 0.01;
        public static double diffykI = 0.0;
        public static double diffykD = 0.0;
        public static double armkP = 0.012;
        public static double armkI = 0.02;
        public static double armkD = 0.0005;

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            BackAssembly backAssembly = new BackAssembly(hardwareMap);
            MecanumDrive drive;
            DcMotorEx rightSlide;
            DcMotorEx leftSlide;
            waitForStart();
            while (!isStopRequested()) {
                gamepads.copyStates();
                backAssembly.update();
                backAssembly.getDiffy().setPidCoeffs(diffykP, diffykI, diffykD);
                backAssembly.getBackArm().setPidCoeffs(armkP, armkI, armkD);

                if (gamepads.isPressed("dpad_right")){
                    drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -0.2,
                                    -0.2
                            ),
                            -0.0
                    ));
                }
                if (gamepads.isPressed("dpad_left")){
                    rightSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
                    leftSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                }


                if(gamepads.isPressed("circle")) backAssembly.setTargetPreset(Preset.FOLDED);
                if(gamepads.isPressed("square")) backAssembly.setTargetPreset(Preset.TRANSFER);
                if(gamepads.isPressed("dpad_right")) backAssembly.setTargetPreset(Preset.ABOVE_FLOOR);
                if(gamepads.isPressed("dpad_down")) backAssembly.setTargetPreset(Preset.FLOOR);

                if(gamepads.isPressed("right_bumper")) backAssembly.getDiffy().rollGrabber(10);
                if(gamepads.isPressed("left_bumper")) backAssembly.getDiffy().rollGrabber(-10);

                telemetry.addData("Preset", backAssembly.getCurrentPreset());
                telemetry.addData("Target Preset", backAssembly.getTargetPreset());
                telemetry.addLine(backAssembly.toString());
                telemetry.update();
            }
        }
    }
}

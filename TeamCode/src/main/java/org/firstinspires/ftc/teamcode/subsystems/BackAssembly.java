package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BackAssembly {
    private final BackArm backArm;
    private final Diffy diffy;

    private Preset currentPreset;
    private Preset targetPreset;

    public BackAssembly(HardwareMap hmap) {
        backArm = new BackArm(hmap);
        diffy = new Diffy(hmap);
        currentPreset = Preset.FOLDED;
        targetPreset = Preset.FOLDED;
    }

    public enum Preset {
        FOLDED(0, 0),
        TRANSFER(45, 0),
        MIDWAY(93, 90),
        CHILL_GUY(160, 110),
        ABOVE_FLOOR(170, 190),
        FLOOR(177, 240);

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
        if (!backArm.isAtTarget() || !diffy.isAtTarget()) return;
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

    public boolean atTarget(){
        return backArm.isAtTarget() && diffy.isAtTarget();
    }

    @NonNull
    public String toString(){
        return diffy.log() + "\n" + backArm;
    }


    @TeleOp(name = "Back Assembly")
    public static class BackAssemblyOpmode extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
            BackAssembly backAssembly = new BackAssembly(hardwareMap);

            waitForStart();
            while (!isStopRequested()) {
                gamepads.copyStates();
                backAssembly.update();

                if(gamepads.isPressed("circle")) backAssembly.setTargetPreset(Preset.FOLDED);
                if(gamepads.isPressed("square")) backAssembly.setTargetPreset(Preset.TRANSFER);
                if(gamepads.isPressed("dpad_right")) backAssembly.setTargetPreset(Preset.ABOVE_FLOOR);
                if(gamepads.isPressed("dpad_down")) backAssembly.setTargetPreset(Preset.FLOOR);

                telemetry.addData("Preset", backAssembly.getCurrentPreset());
                telemetry.addData("Target Preset", backAssembly.getTargetPreset());
                telemetry.addLine(backAssembly.toString());
                telemetry.update();
            }
        }
    }
}

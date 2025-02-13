package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleEffects {
    public static Gamepad.RumbleEffect leftBuzz = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.0, 400)
            .build();
    public static Gamepad.RumbleEffect rightBuzz = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.0, 400)
            .build();
    public static Gamepad.RumbleEffect alternating = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.25, 150)
            .addStep(0.25, 1.0, 150)
            .addStep(1.0, 0.25, 150)
            .addStep(0.25, 1.0, 150)
            .build();
}

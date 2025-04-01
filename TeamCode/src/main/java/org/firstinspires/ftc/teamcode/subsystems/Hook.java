package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook {
    private final Servo hookServo;
    private HookPosition hookPosition;

    public enum HookPosition {
        UP(1.0),
        DOWN(0.0),
        STANDBY(0.5);

        private final double position;

        HookPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public Hook(Servo hookServo) {
        this(hookServo, HookPosition.STANDBY);
    }

    public Hook(Servo hookServo, HookPosition position) {
        this.hookServo = hookServo;
        setPosition(position);
    }

    public void setPosition(HookPosition position) {
        hookPosition = position;
        hookServo.setPosition(position.getPosition());
    }

    public HookPosition getPosition() {
        return hookPosition;
    }
}

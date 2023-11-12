package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class turnServo {
    Servo turnServ;

    public turnServo(HardwareMap hw) {
        turnServ = hw.get(Servo.class, "Turn");
    }

    public class turnH implements Action {
        @Override
        public boolean run(TelemetryPacket t) {
            double f = turnServ.getPosition();
            t.put("Pos", f);
            return f < .95;
        }

    }

    public Action turnStuff() {
        turnServ.setPosition(1);
        turnServ.setPosition(.95);
        return new turnH();

    }
}

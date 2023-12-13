package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Brat {
    public static Servo bratS;
    public static Servo bratD;

    public Brat(HardwareMap hardwareMap) {
        bratS = hardwareMap.get(Servo.class, "bratStanga");
        bratD = hardwareMap.get(Servo.class, "bratDreapta");
    }

    static public void pozitie(double poz) {
        bratS.setPosition(poz);
        bratD.setPosition(poz);
    }


}
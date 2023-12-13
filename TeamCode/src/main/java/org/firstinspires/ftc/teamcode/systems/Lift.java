package org.firstinspires.ftc.teamcode.systems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Lift {
    static public DcMotorEx glisiera1,glisiera2;

    ///variabile PD lift
    static double liftout=0;
    static double liftencoderPosition=0;
    public static int liftTarget=0;
    public static double liftKp=0.020,liftKd=0.00035,liftKi=0; ///was liftKp=0.014,liftKd=0.0002 liftKi=0
    static double liftlastError=0;
    static double liftlastTarget=0;
    static double lifterror;
    static double liftderivative;
    static double liftintegralSum=0;
    static ElapsedTime lifttimer = new ElapsedTime();
    ///

    public Lift(HardwareMap hardwareMap) {
        glisiera1 = hardwareMap.get(DcMotorEx.class, "glisiera1");
        glisiera2 = hardwareMap.get(DcMotorEx.class, "glisiera2");
        glisiera1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glisiera2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glisiera1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glisiera1.setDirection(DcMotor.Direction.REVERSE);
        glisiera2.setDirection(DcMotor.Direction.FORWARD);
    }

    static public void LiftUpdate() {
        int pozglisiera;
        pozglisiera = glisiera1.getCurrentPosition();
        if (pozglisiera!=liftTarget) {
            if (liftTarget != liftlastTarget) {
                liftintegralSum = 0;
            }
            liftencoderPosition = pozglisiera;
            lifterror = liftTarget - liftencoderPosition;
            liftderivative = (lifterror - liftlastError) / lifttimer.seconds();
            liftintegralSum = liftintegralSum + (lifterror * lifttimer.seconds());
            liftout = (liftKp * lifterror) + (liftKi * liftintegralSum) + (liftKd * liftderivative);
            glisiera1.setPower(liftout);
            glisiera2.setPower(liftout);
            liftlastError = lifterror;
            lifttimer.reset();
            liftlastTarget=liftTarget;
        }
        else {
            glisiera1.setPower(0);
            glisiera2.setPower(0);
        }
    }

}
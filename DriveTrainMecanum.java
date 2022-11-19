package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;

public class DriveTrainMecanum {
    double tics_per_rev = 384.5, wheel_rad = 0.05;
    DcMotor MotorRF, MotorLF, MotorRB, MotorLB, Lift;
    double[] oldMotorEnc = {0, 0, 0, 0};
    double[] curMotorEnc = {0, 0, 0, 0};
    double[] deltaMotorEnc = {0, 0, 0, 0};
    double[] basePos = {0, 0, 0, 0};
    vec2 curPosField = new vec2(0), deltaPos = new vec2(0);

    /*
    # Name
    0 Left Back
    1 Left Forward
    2 Right Forward
    3 Right Back
     */

    public void init(HardwareMap HM){
        MotorLB = HM.get(DcMotor.class, "motor_lb");
        MotorRB = HM.get(DcMotor.class, "motor_rb");
        MotorLF = HM.get(DcMotor.class, "motor_lf");
        MotorRF = HM.get(DcMotor.class, "motor_rf");
        Lift = HM.get(DcMotor.class, "lift");

        MotorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        basePos = getEncoders();
    }

    public void applySpeed(double x, double y, double rotate, double koef){
        double lf = y + x + rotate;
        double lb = y - x + rotate;
        double rf = y - x - rotate;
        double rb = y + x - rotate;
        double[] powers = {lf, lb, rf, rb};
        double dev = 0.;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(powers[i]) > dev)
                dev = Math.abs(powers[i]);
        }

        if(dev > 1){
            lf /= dev;
            lb /= dev;
            rf /= dev;
            rb /= dev;
        }

        MotorLB.setPower(lb / koef);
        MotorRB.setPower(rb / koef);
        MotorLF.setPower(lf / koef);
        MotorRF.setPower(rf / koef);
    }

    public void applyBreak(){
        MotorLB.setPower(0);
        MotorRB.setPower(0);
        MotorLF.setPower(0);
        MotorRF.setPower(0);
    }

    public double[] getEncoders(){
        curMotorEnc[0] = (MotorLB.getCurrentPosition() - basePos[0]) / tics_per_rev * 2 * PI;
        curMotorEnc[1] = (MotorLF.getCurrentPosition() - basePos[1]) / tics_per_rev * 2 * PI;
        curMotorEnc[2] = (MotorRF.getCurrentPosition() - basePos[2]) / tics_per_rev * 2 * PI;
        curMotorEnc[3] = (MotorRB.getCurrentPosition() - basePos[3]) / tics_per_rev * 2 * PI;
        return curMotorEnc;
    }

    public vec2 getPos(double angle){
        updatePos(angle);
        return curPosField;
    }

    public void updatePos(double angle){
        oldMotorEnc = curMotorEnc;
        getEncoders();
        for(int i = 0; i < 4; i++) deltaMotorEnc[i] = curMotorEnc[i] - oldMotorEnc[i];
        deltaPos.Y = (deltaMotorEnc[0] + deltaMotorEnc[1] + deltaMotorEnc[2] + deltaMotorEnc[3]) * wheel_rad / sqrt(2) / 4.;
        deltaPos.X = ((deltaMotorEnc[1] + deltaMotorEnc[3]) - (deltaMotorEnc[0] + deltaMotorEnc[2])) * wheel_rad / sqrt(2) / 4.;
        curPosField.plus(deltaPos.turn(angle));
    }
}

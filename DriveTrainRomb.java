package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainRomb extends org.firstinspires.ftc.teamcode.modules.Module {
    private DcMotor rf, rr, lf, lr;
    private static double diffAngle = 45;

    public DriveTrainRomb (){

    }

    @Override
    public void init(HardwareMap hwd) {
        lf = hwd.get(DcMotor.class, "lf");
        lr = hwd.get(DcMotor.class, "lr");
        rf = hwd.get(DcMotor.class, "rf");
        rr = hwd.get(DcMotor.class, "rr");
        //lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void setPower(double plf, double plr, double prf, double prr, double k){
        lf.setPower(plf * k);
        lr.setPower(plr * k);
        rf.setPower(prf * k);
        rr.setPower(prr * k);
    }

    public void stop (){
        setPower(0, 0,0,0,0);
    }

    public double[] getPosition(){
        double sin = Math.sin(Math.toRadians(-45));
        double cos = Math.cos(Math.toRadians(-45));

        int x = lf.getCurrentPosition();
        int y = rf.getCurrentPosition();
        double x1 = y * sin + x * cos;
        double y1 = y * cos + x * sin;

        double xi = (x1 / 1120) * 2 * 5 * Math.PI;
        double yi = (y1 / 1120) * 2 * 5 * Math.PI;
        double position[] = {xi, yi};
        return position;
    }

    public void follow (double x, double y, double t){
        double sin = Math.sin(Math.toRadians(45));
        double cos = Math.cos(Math.toRadians(45));

        double x1 = y * sin + x * cos;
        double y1 = y * cos + x * sin;

        //
        // t = t / Math.abs(t);
        double len = Math.sqrt(y * y + x * x);

        if (len < 0.02){
            setPower(-t, -t, t, t, 0.3);
        }else{
            setPower(x1 - t, y1 - t, x1 + t, y1 + t, 0.5);
        }
    }
}

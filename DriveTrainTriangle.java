package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainTriangle extends org.firstinspires.ftc.teamcode.modules.Module {
    private DcMotor motor_left_back, motor_forward, motor_right_back, lift, intake, turner;
    public DriveTrainTriangle (){

    }
    @Override
    public void init(HardwareMap hwd) {
        motor_left_back = hwd.get(DcMotor.class, "motor_left_back");
        motor_forward = hwd.get(DcMotor.class, "motor_forward");
        motor_right_back = hwd.get(DcMotor.class, "motor_right_back");
        lift = hwd.get(DcMotor.class, "lift");
        intake = hwd.get(DcMotor.class, "intake");
        turner = hwd.get(DcMotor.class, "turner");

        motor_left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_forward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_forward.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_left_back.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        turner.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    public void setPowerToFollow(double forw, double lb, double rb, double k){
        motor_forward.setPower(forw * k);
        motor_left_back.setPower(lb * k);
        motor_right_back.setPower(rb * k);
    }

    public void setPowerToLift(double liftpower){
        lift.setPower(liftpower);
    }


    public void stop (){
        setPowerToFollow(0, 0,0, 0);
    }

    public double getPositionOfLift(){

        int angle = lift.getCurrentPosition();



        double position_lift = angle;
        return position_lift;
    }



    public void follow (double x, double y, double t){
        //double x1 = y * sin + x * cos;
        //double y1 = y * cos + x * sin;

        double yforw = -y;
        double len = Math.sqrt(y * y + x * x);

        if (Math.abs(y) >= Math.abs(x) && y != 0) {
            yforw = -y;
        }

        if (len < 0.02){
            setPowerToFollow(-t, t, t, 0.3);
        }else{
            setPowerToFollow(-yforw, (y + Math.sqrt(3) * x) / 2, (y - Math.sqrt(3) * x) / 2, 0.5);
        }
    }
    }


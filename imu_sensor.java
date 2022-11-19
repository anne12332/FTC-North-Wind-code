package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class imu_sensor {
    static double startAng;
    private double ang, positiveAng;
    BNO055IMU imu;
    Orientation angles = new Orientation();
//    Acceleration accel;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    vec2 accel_vec;

    public void init(HardwareMap HM) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        imu = HM.get(BNO055IMU.class, "imu");

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated());
        resetAngle();
    }

    public void resetAngle() {
        startAng = getAngle();
    }

    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        ang = angles.firstAngle - startAng;
        if (ang < -PI) ang += 2 * PI;
        if (ang > PI) ang -= 2 * PI;
        return ang;
    }

    public double getPositiveAngle() {
        positiveAng = getAngle();
        if (positiveAng < 0) positiveAng += 2 * PI;
        return positiveAng;
    }
/*
    public void startAccel(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public vec2 getAccel(){
        accel = imu.getGravity();
        return accel_vec.set(accel.xAccel, accel.yAccel);
    }*/
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.TensorFlow;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotNW {
    DriveTrainMecanum WB = new DriveTrainMecanum();
    Servo servo;
    imu_sensor IMU = new imu_sensor();
    TensorFlow camera = null;
    ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap HM, Telemetry tele, LinearOpMode lop){
        WB.init(HM);
        IMU.init(HM);
        servo = HM.get(Servo.class, "servo");
        camera = new TensorFlow(HM,  lop,  tele, "model_unquant.tflite", "labels.txt");

    }

    public void waiter(long time){
        timer.reset();
        while(timer.milliseconds() < time);
    }

    public vec2 getPos() {
        return WB.getPos(IMU.getAngle());
    }
}

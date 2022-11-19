package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainTele")
public class MainTele extends LinearOpMode {
    RobotNW Robot = new RobotNW();
    double[] basePos = {0, 0, 0, 0};

    public void runOpMode() {
        Robot.init(hardwareMap, telemetry, this);
        telemetry.addData("", "Init complited");
        telemetry.update();
        basePos = Robot.WB.getEncoders();

        waitForStart();

        while (opModeIsActive()) {
            double koef = 1.;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotate = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepad1.a) koef = 2.5;
            else if (gamepad1.b) koef = 5. / 3.;
            else koef = 1.;

            Robot.WB.applySpeed(x, y, rotate, koef);

            if (gamepad2.b) {
                Robot.servo.setPosition(0.16);
            } else if (gamepad2.a) {
                Robot.servo.setPosition(0.4);
            } else if (gamepad2.x) {
                Robot.servo.setPosition(0.8);
            } else if (gamepad2.right_trigger > 0.1) {
                Robot.servo.setPosition(0.8 - gamepad2.right_trigger * 0.8);
            }

            telemetry.addData("Positive Angle", Robot.IMU.getPositiveAngle());
            telemetry.addData("Angle", Robot.IMU.getAngle());
            telemetry.addData("lb", Robot.WB.MotorLB.getCurrentPosition() - basePos[0]);
            telemetry.addData("lf", Robot.WB.MotorLF.getCurrentPosition() - basePos[1]);
            telemetry.addData("rf", Robot.WB.MotorRF.getCurrentPosition() - basePos[2]);
            telemetry.addData("rb", Robot.WB.MotorRB.getCurrentPosition() - basePos[3]);
            telemetry.addData("Pos X", Robot.getPos().X);
            telemetry.addData("Pos Y", Robot.getPos().Y);
            telemetry.update();
        }

        //telemetry.addData("x", Robot.WB.getPos());
        //telemetry.addData("y", Robot.WB.getPos());
        //telemetry.update();
    }
}

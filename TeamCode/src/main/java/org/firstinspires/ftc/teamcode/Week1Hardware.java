package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Week1Hardware
{
    public DcMotor motorL;
    public DcMotor motorR;

    public DistanceSensor sensorDistance;
    public ColorSensor sensorColor;
    public DigitalChannel sensorTouch;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public Week1Hardware(){}

    public void Init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        motorL = hwMap.get(DcMotor.class, "motor_l");
        motorR = hwMap.get(DcMotor.class, "motor_r");

        sensorDistance = hwMap.get(DistanceSensor.class, "sensorDistance");
        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");
        sensorTouch = hwMap.get(DigitalChannel.class, "sensorTouch");

        motorR.setDirection(DcMotor.Direction.FORWARD);
        motorL.setDirection(DcMotor.Direction.REVERSE);

        motorR.setPower(0);
        motorL.setPower(0);

        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorTouch.setMode(DigitalChannel.Mode.INPUT);
    }
}

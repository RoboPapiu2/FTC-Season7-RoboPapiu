package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

public class hardwarePapiu {
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;
    public DcMotor bratz = null;
    public Servo servoRight, servoLeft;
    public DigitalChannel digitalTouch;
    HardwareMap hwMap = null;
    public hardwarePapiu(){}
    public void init(HardwareMap ahwmap){
        hwMap=ahwmap;

        /* Motoare Baza */

        bratz = hwMap.get(DcMotor.class, "motorZ");
        bratz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoLeft = hwMap.get(Servo.class, "servoLeft");
        servoRight = hwMap.get(Servo.class, "servoRight");
        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.REVERSE);

        digitalTouch = hwMap.get(DigitalChannel.class, "touchSensor");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }
    public void EncoderReset() {
        bratz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bratz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
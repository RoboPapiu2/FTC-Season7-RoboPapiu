package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class hardwareTesting {
    //C = Control Hub, E = Expansion Hub
    // Initiate hashmap for motors and servos
    HashMap<Integer, DcMotor> MotorMap = new HashMap<Integer, DcMotor>();
    HashMap<Integer, Servo> ServoMap = new HashMap<Integer, Servo>();

    /** Motors */
    DcMotor EMotor0, EMotor1, EMotor2, EMotor3;
    DcMotor CMotor0, CMotor1, CMotor2, CMotor3;
    /** Servos */
    Servo EServo0, EServo1, EServo2, EServo3;
    Servo CServo0, CServo1, CServo2, CServo3;

    HardwareMap hwMap = null;
    public hardwareTesting(){}
    public void init(HardwareMap ahwmap) {
        MotorMap.put(0, EMotor0);
        MotorMap.put(1, EMotor1);
        MotorMap.put(2, EMotor2);
        MotorMap.put(3, EMotor3);
        MotorMap.put(4, CMotor0);
        MotorMap.put(5, CMotor1);
        MotorMap.put(6, CMotor2);
        MotorMap.put(7, CMotor3);
        EMotor0 = hwMap.get(DcMotor.class, "EMotor0");
        EMotor1 = hwMap.get(DcMotor.class, "EMotor1");
        EMotor2 = hwMap.get(DcMotor.class, "EMotor2");
        EMotor3 = hwMap.get(DcMotor.class, "EMotor3");
        CMotor0 = hwMap.get(DcMotor.class, "CMotor0");
        CMotor1 = hwMap.get(DcMotor.class, "CMotor1");
        CMotor2 = hwMap.get(DcMotor.class, "CMotor2");
        CMotor3 = hwMap.get(DcMotor.class, "CMotor3");

        ServoMap.put(0, EServo0);
        ServoMap.put(1, EServo1);
        ServoMap.put(2, EServo2);
        ServoMap.put(3, EServo3);
        ServoMap.put(4, CServo0);
        ServoMap.put(5, CServo1);
        ServoMap.put(6, CServo2);
        ServoMap.put(7, CServo3);
        CServo0 = hwMap.get(Servo.class, "CServo0");
        CServo1 = hwMap.get(Servo.class, "CServo1");
        CServo2 = hwMap.get(Servo.class, "CServo2");
        CServo3 = hwMap.get(Servo.class, "CServo3");
        EServo0 = hwMap.get(Servo.class, "EServo0");
        EServo1 = hwMap.get(Servo.class, "EServo1");
        EServo2 = hwMap.get(Servo.class, "EServo2");
        EServo3 = hwMap.get(Servo.class, "EServo3");
    }
    int servoIncrementLvl = 1;
    public double setIncrements(int PM){
        //PM - plus minus, 0 - minus, 1 - plus
        if(PM==1) servoIncrementLvl++;
        else if(PM==0) servoIncrementLvl--;
        //Limit value
        if(servoIncrementLvl<1) servoIncrementLvl=1;
        if(servoIncrementLvl>4) servoIncrementLvl=4;
        double A=0.5;
        switch (servoIncrementLvl) {
            case 1:
                A = 0.5;
                break;
            case 2:
                A = 0.25;
                break;
            case 3:
                A = 0.1;
                break;
            case 4:
                A = 0.01;
                break;
        }
        return A;
    }
}

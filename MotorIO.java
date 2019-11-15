package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorIO implements Runnable{

    Telemetry telemetry;
    HardwareMap hwmap;
    
    boolean kys_signal = false;
    double motor_a_power = 0;
    double motor_b_power = 0;
    double motor_c_power = 0;
    double motor_d_power = 0;
    
    double motor_0_power = 0;
    double motor_1_power = 0;
    double motor_2_power = 0;
    double motor_3_power = 0;
    
    DcMotorEx motor_a;
    DcMotorEx motor_b;
    DcMotorEx motor_c;
    DcMotorEx motor_d;
    
    DcMotorEx motor_0;
    DcMotorEx motor_1;
    DcMotorEx motor_2;
    DcMotorEx motor_3;
    
    Servo servo_0;
    Servo servo_1;
    
    double servo_0_position = 0;
    double servo_1_position = 0;
    double rotation = 0;

    public void kys(){
        kys_signal = true;
    }
    
    public MotorIO(Telemetry telemetry, HardwareMap hwmap){
        this.telemetry = telemetry;
        this.hwmap = hwmap;
    }
    
    public void updateMotors(double motor_a_power, double motor_b_power, double motor_c_power, double motor_d_power){
        this.motor_a_power = motor_a_power;
        this.motor_b_power = motor_b_power;
        this.motor_c_power = motor_c_power;
        this.motor_d_power = motor_d_power;
    }
    public void setRotation(double rotation){
        this.rotation = rotation;
    }
    public double[] getPositions(){
        return new double[]{this.motor_0.getCurrentPosition(), this.motor_1.getCurrentPosition(), this.motor_2.getCurrentPosition(), this.motor_3.getCurrentPosition()};
    }
    public void updateMotor(double motor_power, int index){
        if(index == 0){
            this.motor_0_power = motor_power;
        }
        else if(index == 1){
            this.motor_1_power = motor_power;
        }
        else if(index == 2){
            this.motor_2_power = motor_power;
        }
        else if(index == 3){
            this.motor_3_power = motor_power;
        }
    }
    public void setServo(double position, int index){
        if(index == 0){
            this.servo_0_position = position;
        }
        else if (index == 1){
            this.servo_1_position = position;
        }
    }
    @Override
    public void run(){
        motor_a = hwmap.get(DcMotorEx.class, "lAw"); // left A wheel
        motor_b = hwmap.get(DcMotorEx.class, "lBw"); // left B wheel
        motor_c = hwmap.get(DcMotorEx.class, "rCw"); // right C wheel
        motor_d = hwmap.get(DcMotorEx.class, "rDw"); // right D wheel

        motor_0 = hwmap.get(DcMotorEx.class, "ly"); // left yeet
        motor_1 = hwmap.get(DcMotorEx.class, "ry"); // right yeet
        motor_2 = hwmap.get(DcMotorEx.class, "ln"); // left nostin
        motor_3 = hwmap.get(DcMotorEx.class, "rn"); // right nostin

        motor_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo_0 = hwmap.get(Servo.class, "ls"); // left servo
        servo_1 = hwmap.get(Servo.class, "rs"); // right servo

        while(!kys_signal){
            motor_a.setPower(motor_a_power + rotation);
            motor_b.setPower(motor_b_power + rotation);
            motor_c.setPower(motor_c_power + rotation);
            motor_d.setPower(motor_d_power + rotation);

            motor_0.setVelocity(motor_0_power * 1000);
            motor_1.setVelocity(motor_1_power * 1000);

            motor_2.setVelocity(motor_2_power * 1000);
            motor_3.setVelocity(motor_3_power * 1000);

            servo_0.setPosition(servo_0_position);
            servo_1.setPosition(servo_1_position);
        }
    }
}
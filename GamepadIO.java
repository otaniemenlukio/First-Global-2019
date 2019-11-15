package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.io.Console;

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

public class GamepadIO implements Runnable{
    Telemetry telemetry;
    HardwareMap hwmap;
    Gamepad gamepad1;
    MotorIO motorIO;
    Pid pid;
    double extend;
    
    boolean kys_signal = false;
    
    long bumperMillis = 0;
    boolean bumper_open = false;
    
    public void kys(){
        this.kys_signal = true;
    }
    public double magnitudeFromVector(double[] vector){
        return Math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]);
    }
    public double angleFromVector(double[] vector){
        double x = vector[0];
        double y = vector[1];
        if(x == 0){
            if(y == 0){
                return 0.0;
            }
            else if(y > 0){
                return Math.PI/2; //90
            }
            else{
                return Math.PI*3/2; //270
            }
        }
        else if(y == 0){
            if(x > 0){
                return 0.0;
            }
            else{
                return Math.PI; //180
            }
        }
        if(x > 0){
            if(y > 0){
                return Math.atan(y/x);
            }
            else{
                return Math.PI*2+Math.atan(y/x);
            }
        }
        else{
            return Math.PI+Math.atan(y/x);
        }
    }
    
    public double[] vectorFromAngle(double angle, double magnitude){
        return new double[]{Math.cos(angle)*magnitude,Math.sin(angle)*magnitude};
    }
    
    public GamepadIO(Telemetry telemetry, HardwareMap hwmap, Gamepad gamepad1, MotorIO motorIO, Pid pid){
        this.telemetry = telemetry;
        this.hwmap = hwmap;
        this.gamepad1 = gamepad1;
        this.motorIO = motorIO;
        this.pid = pid;
    }
    
    @Override
    public void run(){
        double x;
        double y;
        double rot;
        double angle;
        double magnitude;
        
        
        double[] raw_vector;
        double[] vector;
        double extend_power = 1.5;
        double yeet_power = 1;
        double speed = 1;
        double rot_turbo = 1;
        
        
        double servo_0_position = 0.0;
        double servo_1_position = 0.0;

        boolean init = true;
        boolean right_bumper_released = true;
        boolean left_bumper_released = true;

        boolean doksista = false; // boolean to toggle dock mode
        boolean dpad_down_released = true;

        while(!kys_signal){
            try{
                double tickMillis = System.currentTimeMillis();
                
                x = -gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
                
                if(doksista){
                    motorIO.updateMotor(-5,0);
                    motorIO.updateMotor(5,1);
                }
                else{
                    if(gamepad1.left_bumper){
                        telemetry.addLine("right");
                    
                        motorIO.updateMotor(yeet_power,0);
                        motorIO.updateMotor(-yeet_power,1);
                    }
                    else if(gamepad1.right_bumper){
                        telemetry.addLine("left");
                    
                        motorIO.updateMotor(-yeet_power,0);
                        motorIO.updateMotor(yeet_power,1);
                    }
                    else{
                        motorIO.updateMotor(0.0,0);
                        motorIO.updateMotor(0.0,1);
                    }
                }
                if(gamepad1.dpad_down && dpad_down_released){
                    dpad_down_released = false;
                    doksista = !doksista;
                }
                if(!gamepad1.dpad_down){
                    dpad_down_released = true;
                }
                if(doksista){
                    extend_power = 10.0;
                }    
                else{
                    if(gamepad1.left_trigger){
                        extend_power = 1.0;
                    }
                    else if(gamepad1.right_trigger){
                        extend_power = -1.0;
                    }
                    else{
                        extend_power = 0.0;
                    }
                }
                if(gamepad1.x){
                    speed = 3;
                    rot_turbo = 2;
                }
                if(gamepad1.y){
                    speed = 1.5;
                    rot_turbo = 1;
                }
                if(gamepad1.a){
                    pid.enable();
                    pid.setTarget();
                }
                if(gamepad1.b){
                    pid.disable();
                }
                
                
                
                if(!gamepad1.dpad_up){
                    left_bumper_released = true;
                }
                if(gamepad1.dpad_up && left_bumper_released || init){
                    left_bumper_released = false;
                    if(servo_1_position == 0){
                        servo_0_position = 0;
                        bumper_open = true;
                    }
                    else{
                        servo_1_position = 0;
                        bumper_open = false;
                    }
                    bumperMillis = System.currentTimeMillis();
                    init = false;
                }
                
                telemetry.addData("bumper_open", bumper_open);
                telemetry.addData("bumperMillis", bumperMillis);
                if(bumperMillis != 0 && System.currentTimeMillis() - bumperMillis > 350){
                    if(bumper_open){
                        servo_1_position = 1;
                    }else{
                        servo_0_position = 1;
                    }
                    bumperMillis = 0;
                }
                
                raw_vector = new double[]{x, y};
                angle = angleFromVector(raw_vector) + Math.PI/4;
                magnitude = magnitudeFromVector(raw_vector) * 0.5 * speed;
                
                vector = vectorFromAngle(angle, magnitude);
                rot = gamepad1.right_stick_x * 0.3 * rot_turbo;
                
                if(x * x + y * y < 0.02 && Math.abs(rot) < 0.05){
                    motorIO.updateMotors(0.1, 0.1, -0.1, -0.1);
                }
                else{
                    motorIO.updateMotors(vector[0] + rot, -vector[1] + rot, -vector[0] + rot, vector[1] + rot);
                }
                motorIO.updateMotor(extend_power, 2);
                motorIO.updateMotor(-extend_power, 3);
                motorIO.setServo(servo_0_position, 0);
                motorIO.setServo(servo_1_position, 1);

                
                Thread.sleep(50);
            }
            catch(InterruptedException e){
                telemetry.addData("Error", e);
                telemetry.update();
            }
        }
    }
    
}
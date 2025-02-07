package org.firstinspires.ftc.teamcode.hardware;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double lastError;
    private double lastTime;
    private double maxkI;
    private double integralSum;
    public PIDController(double kP,double kI,double kD,double maxkI){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxkI = maxkI;
    }
    public double calculation(double target, double current, double time){
        //PID calculation variables
        double error = target - current;
        double deltaTime = (time - lastTime);
        double output = 0;

        //avoid division by zero in dOutPut
        if(time == 0){
            lastTime = time;
            lastError = error;
            return error*kP;
        }

        //calculate PID values
        double pOutput = error*kP;

        double dOutput = ((error-lastError)/deltaTime)*kD;
        integralSum += error * deltaTime;
        integralSum = Math.max(-maxkI, Math.min(maxkI, integralSum));

        double iOutput = integralSum * kI;
        //output value for PID
        output = pOutput + iOutput + dOutput;

        lastError = error;
        lastTime = time;

        return Math.max(-1, Math.min(1,output));
    }
}

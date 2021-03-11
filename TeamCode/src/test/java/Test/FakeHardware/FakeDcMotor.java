package Test.FakeHardware;


import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;

// Does not extend DcMotor
// Need to return a MotorConfigurationType object
public class FakeDcMotor {

    double power = 0.0;
    double ticks = 0;
    Motors motor;
    private double time = 0.0;

    FakeDcMotor(Motors motor) {
        this.motor = motor;
    }

    // Fake Physics - basic acceleration limit
    private final double maxTicksPerSecond = 2300;            // Telemetry suggests velocity of 2500 ticks/second
    private final double maxTicksPerSecondPerSecond = 6000;   // Telemetry suggests acceleration of 9000 ticks/s/s
    private double currentTicksPerSecond = 0;

    public void updateAndIntegratePosition(double time) {
        double deltaTime = time - this.time;
        if(deltaTime == 0.0) return;

        // Limit Acceleration
        double desiredTicksPerSecond = power * maxTicksPerSecond;
        double desiredAcceleration_TicksPerSecPerSec = (desiredTicksPerSecond - currentTicksPerSecond) / deltaTime;
        if(Math.abs(desiredAcceleration_TicksPerSecPerSec) > maxTicksPerSecondPerSecond) {
            double accelerationSign = desiredAcceleration_TicksPerSecPerSec / Math.abs(desiredAcceleration_TicksPerSecPerSec);
            // Limit the acceleration
            currentTicksPerSecond += accelerationSign * maxTicksPerSecondPerSecond * deltaTime;
//            System.out.println("ACCELERATION LIMIT ACTIVATED");
        } else {
            currentTicksPerSecond = desiredTicksPerSecond;
        }

        // Limit Velocity
        if(Math.abs(currentTicksPerSecond) > maxTicksPerSecond) {
            currentTicksPerSecond *= Math.abs(maxTicksPerSecond/currentTicksPerSecond);
        }

        this.ticks += currentTicksPerSecond * deltaTime;
        this.time = time;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }

    public void setTicks(int ticks) {
        this.ticks = ticks;
    }

    public double getTicks() {
        return ticks;
    }
}
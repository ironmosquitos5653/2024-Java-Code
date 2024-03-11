package frc.generator.data;

public class WaitAutoCommand implements AutoCommand {
    private double waitTime;

    public WaitAutoCommand(double waitTime) {
        this.waitTime = waitTime;
    }

    public double getWaitTime() {
        return waitTime;
    }

    @Override
    public void generate(StringBuilder sb, int indent) {
        sb.append("new WaitCommand(" + waitTime + ")");
    }
}
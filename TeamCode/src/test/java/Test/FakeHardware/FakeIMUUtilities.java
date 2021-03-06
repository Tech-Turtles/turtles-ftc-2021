package Test.FakeHardware;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Odometry.IMUUtilities;

public class FakeIMUUtilities extends IMUUtilities {

    // Default to Fast, Heading only mode when not specified.
    public FakeIMUUtilities(RobotHardware opMode, String imu_name) {
        this(opMode,imu_name, ImuMode.FAST_HEADING_ONLY);
    }

    public FakeIMUUtilities(RobotHardware opMode, String imu_name, ImuMode imuMode) {
        super(opMode,imu_name,imuMode);

    }

//    public FakeIMUUtilities() {}

}

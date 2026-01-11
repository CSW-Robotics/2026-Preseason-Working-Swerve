
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

    private String m_network_table_key = "limelight";

    // offsets for the camera
    public double xoffset;
    public double zoffset;
    public double rotoffset;

    public LimeLight(String network_table_key,double xoffset, double zoffset, double rotoffset) {
        // sets the key for the limelight aka the name 
        m_network_table_key = network_table_key;

        // stores the offsets
        this.xoffset = xoffset;
        this.zoffset = zoffset;
        this.rotoffset = rotoffset;
    }


    // 3d AT data
    public double[] targetData = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    public double[] posData = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("botpose").getDoubleArray(new double[6]);

    public double last_seen_tag_id = 0;

    public void setAprilTag() {
        NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("pipeline").setNumber(0);
    }

    public void setReflective() {
        NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("pipeline").setNumber(1);
    }

    @Override
    public void periodic() {

        targetData = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        posData = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("botpose").getDoubleArray(new double[6]);

    }

    @Override
    public void simulationPeriodic() {}
}
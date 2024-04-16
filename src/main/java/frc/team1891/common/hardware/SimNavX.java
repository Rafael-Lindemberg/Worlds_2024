package frc.team1891.common.hardware;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

public class SimNavX extends NavX {
    private final SimDouble simAngle;
    // public SimNavX(SPI.Port kmxp){
    //     super(kmxp);
    //     int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    //     simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // }

    public SimNavX(SerialPort.Port serial_port_id) {
        super(serial_port_id);
        
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    }


    

    public SimNavX() {
        super();

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    }
    public SimNavX(I2C.Port port) {
        super(port);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    }

    public void setDegrees(double degrees) {
        simAngle.set(degrees);
    }

    public void setRadians(double radians) {
        simAngle.set(Math.toDegrees(radians));
    }
}


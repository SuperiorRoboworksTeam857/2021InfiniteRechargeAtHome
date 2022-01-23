package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pixy2Handler{
    boolean lampOn = false;
    byte[] lastCache = {
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00
    };
    byte[] localCache = {
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00
    };
    //public ArrayList<String> VectorData = new ArrayList<String>();
    public final byte[] CHECKSUM_VERSIONREQUEST = {
        (byte) 0xae,  // first byte of no_checksum_sync (little endian -> least-significant byte first)
        (byte) 0xc1,  // second byte of no_checksum_sync
        (byte) 0x0e,  // this is the version request type
        (byte) 0x00   // data_length is 0
    };

    public final byte[] CHECKSUM_SETLEDCOLOR = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x14,
        (byte) 0x03,
        (byte) 0x00,
        (byte) 0xFF,
        (byte) 0x00
    };

    public final byte[] CHECKSUM_LAMPON = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x16,
        (byte) 0x02,
        (byte) 0x01,
        (byte) 0x01
    };

    public final byte[] CHECKSUM_LAMPOFF = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x16,
        (byte) 0x02,
        (byte) 0x00,
        (byte) 0x00
    };

    public final byte[] CHECKSUM_GETMAINFEATURES = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x30,
        (byte) 0x02,
        (byte) 0x00,
        (byte) 0x07
    };

    public final byte[] CHECKSUM_SETLINEMODE = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x02,
        (byte) 0x05,
        (byte) 0x6C,
        (byte) 0x69,
        (byte) 0x6E,
        (byte) 0x65,
        (byte) 0x00
    };

    public final byte[] CHECKSUM_GETBLOCKS = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x20,
        (byte) 0x02,
        (byte) 0xFF,
        (byte) 0x01
    };

    public final byte[] CHECKSUM_SETMODE = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x36,
        (byte) 0x01,
        (byte) 0x01
    };

    I2C pixy = new I2C(Port.kOnboard, 0x54);
    public boolean ballDetected = true;

    public boolean init(){
        //boolean status = pixy.writeBulk(CHECKSUM_SETLINEMODE);
        //SmartDashboard.putNumber("initializing pixy...");
        //return !status;
        return true; 
    }

    public void toggleLamp(){
        if(lampOn){
            pixy.writeBulk(CHECKSUM_LAMPOFF);
            lampOn = false;
        }else{
            pixy.writeBulk(CHECKSUM_LAMPON);
            lampOn = true;
        }
    }

    public void sendRequest(byte[] byteArray){
        pixy.writeBulk(byteArray);
        byte[] initBuffer =  new byte[20];
        pixy.readOnly(initBuffer, 20);
        //if(initBuffer[10] != -2 && initBuffer[10] != -128){
        if (initBuffer[2] == 33) {
            if (initBuffer[3] == 14) {
                ballDetected = true;
                localCache = initBuffer;
                lastCache = initBuffer;    
            } else {
                ballDetected = false;
                localCache = lastCache;
            }
            
        }
        //}else{
        //    localCache = lastCache;
        //    ballDetected = false;
        //}
        
    }

    public void check(){

    }

    // public byte x0(){
    //     return localCache[8];
    // }
    // public byte y0(){
    //     return localCache[9];
    // }
    // public byte x1(){
    //     return localCache[10];
    // }
    // public byte y1(){
    //     return localCache[11];
    // }

    public int x(){
        return ((localCache[9] << 8) | localCache[8] & 0x00FF);
    }
    public int y(){
        return ((localCache[11] << 8) | localCache[10] & 0x00FF);
    }

    public void printLocalCache(){
        SmartDashboard.putNumber("x", x());
        SmartDashboard.putNumber("y", y());


        SmartDashboard.putNumber("0: ",localCache[0]);
        SmartDashboard.putNumber("1: ",localCache[1]);
        SmartDashboard.putNumber("2: ",localCache[2]);
        SmartDashboard.putNumber("3: ",localCache[3]);
        SmartDashboard.putNumber("4: ",localCache[4]);
        SmartDashboard.putNumber("5: ",localCache[5]);
        SmartDashboard.putNumber("6: ",localCache[6]);
        SmartDashboard.putNumber("7: ",localCache[7]);
        SmartDashboard.putNumber("x0: ",localCache[8]);
        SmartDashboard.putNumber("x1: ",localCache[9]);
        SmartDashboard.putNumber("y0: ",localCache[10]);
        SmartDashboard.putNumber("y1: ",localCache[11]);
        SmartDashboard.putNumber("12: ",localCache[12]);
        SmartDashboard.putNumber("13: ",localCache[13]);
        SmartDashboard.putNumber("14: ",localCache[14]);
        SmartDashboard.putNumber("15: ",localCache[15]);
        SmartDashboard.putNumber("16: ",localCache[16]);
        SmartDashboard.putNumber("17: ",localCache[17]);
        SmartDashboard.putNumber("18: ",localCache[18]);
        SmartDashboard.putNumber("19: ",localCache[19]);
       
    }
}
// ADS Touch Sensor Test Example Program (IC P/N:ANMG15 )
// Code:
// Date: 2023.07.12  Ver.: 0.0.1
// H/W Target: ARDUINO UNO R3, S/W: Arduino IDE  2.1.1
// Author: Park, Byoungbae (yni2yni@hanmail.net)
// Note: More information? Please ,send to e-mail.
// Uno R3, A4:SDA,A5: SCL, Leonardo 2:SDA,3:SCL, Nano V3.0  A4:SDA,A5:SCL
// Register setting values are subject to change without prior notice to improve touch operation.

#include <Wire.h>

#define LF        0x0A //New Line
#define CR        0x0D //Carriage  return
#define SPC       0x20 //Space

#define Ch_enable1   0x01 //Touch Channel Enable/Disable
#define Ch_enable2   0x02 //Touch Channel Enable/Disable
#define Global_ctrl0 0x05  //Global Control Register0

// MFM Control Register
#define MFM_TIME0    0x08 // MFM time control
#define MFM_TIME1    0x09 //MFM time control
#define MFM_TIME2    0x0A //MFM time control
#define MFM_TIME3    0x0B //MFM time control
#define MFM_TIME4    0x0C //MFM time control
#define MFM_TIME5    0x0D //MFM time control
#define MFM_TIME6    0x0E //MFM time control

#define Lerr_percent 0x0F //Low Error Percent
#define Cut_percent  0x10 //Cut Percent
#define Herr_percent 0x11 //High Error Percent
#define Error_count  0x12 //Error Count
#define Touch_count  0x13 //CS Filter
#define Cut_count    0x14 //Cut count
#define MFM_up_count 0x15 //MFM up count
#define Up_hold_count   0x16 //Hold count
#define Down_count   0x17 //down count 
#define CS_tcount    0x18 //CS Filter
#define Cut_cal_htime   0x19 //cut cal hold time
#define Init_cal_spd 0x1A //Inital Calibration speed
#define Max_percent  0x1B //Max Percent
#define Fr_response  0x1C //MFM response
#define Duty_ctrl    0x21 //MFM Duty Control

#define Cal_speed    0x30 //Calibration Speed Control at BF mode
#define Cal_BS_speed 0x31 //Calibration Speed Control at BS mode

#define Global_ctrl3 0x32 //Global Option Control Register1
                          
#define Global_ctrl2 0x33 //Global Option Control Register2
                          //(imp_sel,Single/Multi ,Cal_Hold_time,clock_off)
#define Clock_ctrl   0x34 //Clock Control Register (init_cal_opt, clk_sel, rb_sel)
#define RF_exp_time  0x35 //
#define Global_ctrl1 0x36 //
                          //(response_off_ctrl, response_ctrl, bf_mode, Software Reet)
#define State_count  0x37 //Cal_pre_scale

// Sensitivity level (touch output threshold, Register Value X 0.025% = (1 Step=0.025%)							
#define Sensitivity1  0x38 //ch1,Default: 0x1C X 0.025% = 0.70% (threshold %)
#define Sensitivity2  0x39 //ch2
#define Sensitivity3  0x3A //ch3
#define Sensitivity4  0x3B //ch4
#define Sensitivity5  0x3C //ch5
#define Sensitivity6  0x3D //ch6
#define Sensitivity7  0x3E //ch7
#define Sensitivity8  0x3F //ch8
#define Sensitivity9  0x40 //ch9
#define Sensitivity10 0x41 //ch10
#define Sensitivity11 0x42 //ch11
#define Sensitivity12 0x43 //ch12
#define Sensitivity13 0x44 //ch13
#define Sensitivity14 0x45 //ch14
#define Sensitivity15 0x46 //ch15

#define Sensitiviy_loop 0x47 //
#define Sensitiviy_tune 0x48 //
#define Sensitivity16   0x49 //ch16

#define LDO_cap_ctrl 0x4A //

#define Init_ctrl    0x4C //

// -------------------- IC Test Opt. Reg. -----------------------------------------------
#define RAM_Ctrl     0x59 // IC test option
#define RAM_data     0x5A // IC test option
#define FT_ctrl1     0x5B // IC test option
#define FT_ctrl2     0x5C // IC test option
#define FT_state     0x5D // IC test option
#define FT_cspcntmin 0x5E // IC test option
#define FT_cspcnmax  0x5F // IC test option
#define FT_rndpcntmin   0x60 // IC test option
#define FT_rndpcntmax   0x61 // IC test option

#define Percent_read 0x62 //
#define Count_read   0x63 //

//------------- Register Write lock/unlock --------------------------------------------------
#define Unlock_op_en 0x64 //Register Lock Enable/Disable
#define Reg_unlock   0x65 //Register Lock/unlock 
#define Dread_unlock 0x66 //Register address 0x62h, 0x63h Lock/unlock
#define Sen_init_opt 0x67 //
#define BIST_unlock  0xA9 //BIST(Buli-In Self Test) Register address 0x67h ~0x77h Lock/unlock

#define Texp_ctrl1      0x68 //Select the time base for touch expiration
#define Texp_ctrl2      0x69 //Resolution Option for touch expiration time
#define Texp_ctrl3      0x6A //

#define DTR_ctrl1       0x6B //Set the Touch Percent Limit (bit 7~0)
#define DTR_ctrl2       0x6C //Set the Touch Percent Limit (bit 15~8)
#define DTR_ctrl3       0x6D //DTR Mode

// -------------------- BIST(Built in Self Test) -------------------------------------
#define BIST_Start1     0x6E //BIST(Bulit-in Self Test)
#define BIST_Start2     0x6F //BIST(Bulit-in Self Test)
#define BIST5_ctrl1     0x70 //BIST(Bulit-in Self Test)
#define BIST5_ctrl2     0x71 //BIST(Bulit-in Self Test)
#define BIST7_ctrl1     0x72 //BIST(Bulit-in Self Test)
#define BIST7_ctrl2     0x73 //BIST(Bulit-in Self Test)
#define BIST9_ctrl1     0x74 //BIST(Bulit-in Self Test)
#define BIST9_ctrl2     0x75 //BIST(Bulit-in Self Test)
#define CKTEST_ctrl     0x76 //BIST(Bulit-in Self Test)
#define CKTEST_ctr3     0x77 //BIST(Bulit-in Self Test)
#define BIST_all_seq    0x78 //BIST(Bulit-in Self Test)
#define BIST_sts1       0x79 // Read Only
#define BIST_sts2       0x7A // Read Only
#define BIST_sts3       0x7B // Read Only

#define BIST1_iresult   0x7C // Read Only
#define BIST2_iresult1  0x7D // Read Only
#define BIST2_iresult2  0x7E // Read Only
#define BIST3_iresult1  0x80 // Read Only
#define BIST3_iresult2  0x81 // Read Only
#define BIST5_jitt_num1 0x82 // Read Only
#define BIST5_jitt_num2 0x83 // Read Only
#define BIST6_iresult   0x84 // Read Only
#define BIST7_iresult1  0x85 // Read Only
#define BIST7_iresult2  0x86 // Read Only
#define BIST8_iresult   0x87 // Read Only
#define BIST9_iresult1  0x88 // Read Only
#define BIST9_iresult2  0x89 // Read Only
#define BIST9_iresult3  0x8A // Read Only
#define BIST10_iresult  0x8B // Read Only

#define Alarm2_1        0x8C // Read Only
#define Alarm2_2        0x8D // Read Only
#define Alarm3_1        0x8E // Read Only
#define Alarm3_2        0x8F // Read Only

// -------------- Touch Output Data ---------------------------------------------------
#define Output1  0x2A //Touch Key Output Data Register
#define Output2  0x2B //Touch Key Output Data Register
#define Checksum_ro1    0x2C // Checksum for Ouptput1
#define Checksum_ro2    0x2D //Checksum for Ouptput2
#define Checksum_ctrl   0x94 //Checksum factor
#define Checksum_w      0x95 //
#define Checksum_r      0x96 //

// ------------------- IC Test Opt. Reg. ----------------------------------------------
#define FTC_ctrl        0xAA //
#define Nsys_Ctrl       0xB0 //
#define Asen_En1        0xB1 //
#define Asen_En2        0xB2 //
#define Asen_Ctrl1      0xB3 //
#define Asen_Ctrl2      0xB4 //
#define Asen_Ctrl3      0xB5 //
#define Asen_Ctrl4      0xB6 //
#define Asen_min        0xB7 //
#define Asen_max        0xB8 //
#define Asen_negvalue   0xB9 //
#define End_of_aset     0xBA //
#define Asen_fset_req   0xBB //
#define End_of_aset2    0xBC // Read Only
#define End_of_aset3    0xBD // Read Only

#define Fduty_Cr        0xC0 // 
#define Fduty_Cs1       0xC1 // 
#define Fduty_Cs2       0xC2 //  
#define Fduty_Cs3       0xC3 // 
#define Fduty_Cs4       0xC4 // 
#define Fduty_Cs5       0xC5 // 
#define Fduty_Cs6       0xC6 // 
#define Fduty_Cs7       0xC7 // 
#define Fduty_Cs8       0xC8 // 
#define Fduty_Cs9       0xC9 // 
#define Fduty_Cs10      0xCA // 
#define Fduty_Cs11      0xCB // 
#define Fduty_Cs12      0xCC // 
#define Fduty_Cs13      0xCD // 
#define Fduty_Cs14      0xCE // 
#define Fduty_Cs15      0xCF // 
#define Fduty_Cs16      0xD0 // 

#define Sys_ctrl1       0xD1 //

// ----------- Second threshold Control Register -----------------------------------
#define Sys_ctrl2       0xD2 //
#define Sys_ctrl3       0xD3 //

#define Sys_ctrl3       0xD4 //
#define SA_cs_sts1      0xD5 // Read Only
#define SA_cs_sts2      0xD6 // Read Only
#define SA_MFM_sts1     0xD7 // Read Only
#define SA_MFM_sts2     0xD8 // Read Only

#define DTR_ictrl       0xD9 // DTR (Direct touch reset) Control
#define DTR_ictrl1      0xDA // 
#define DTR_ictrl2      0xDB // 
#define DTR_ictrl3      0xDC // 
#define DTR_ictrl4      0xDD // 
#define DTR_ictrl5      0xDE // 
#define DTR_ictrl6      0xDF // 
#define DTR_ictrl7      0xE0 // 
#define DTR_ictrl8      0xE1 // 
#define DTR_ictrl9      0xE2 // 
#define DTR_ictrl10     0xE3 // 
#define DTR_ictrl11     0xE4 // 
#define DTR_ictrl12     0xE5 // 
#define DTR_ictrl13     0xE6 // 
#define DTR_ictrl14     0xE7 // 
#define DTR_ictrl15     0xE8 // 
#define DTR_ictrl16     0xE9 // 

#define Asen_mm_limit   0xEA // 


// ============= ANMG15 I2C Slave Address ============================//
#define ANMG15_ID_VDD  0x24 //0x48 >>1( 0b0100100 7bit + R/W 1 Bit , 7bit=0x24, 8bit=0x48)
#define ANMG15_ID_GND  0x7C //0xF8 >>1( 0b1111100 7bit + R/W 1 Bit , 7bit=0x7C, 8bit=0xF8)
// ============= ANMG15 I2C Slave Address ============================//

void  Init_ANMG15(void); //Initialize ANMG15

#define RESET_PIN 7 //Reset pin
#define EN_PIN    6 //I2C Enable Pin

void Register_Dump()
{
   byte read_data[1] = {0};

   for (int i = 0; i < 256; i += 16)
   {
      for (int j = 0; j <= 15; j++)
      {
         Wire.beginTransmission(ANMG15_ID_VDD); // sned ic slave address
         Wire.write((i + j));                   // sends register address
         Wire.endTransmission();                // stop transmitting
         Wire.requestFrom(ANMG15_ID_VDD, 1);    // data read 
         read_data[0] = Wire.read();            //
         print2hex(read_data, 1);               //
      }
      Serial.write(LF);
      Serial.write(CR);
   }
   delay(500);
}

void print2hex(byte *data, byte length) //Print Hex code
{
   Serial.print("0x");
   for (int i = 0; i < length; i++)
   {
      if (data[i] < 0x10)
      {
         Serial.print("0");
      }
      Serial.print(data[i], HEX);
      Serial.write(SPC);
   }
}

void setup(){
  delay(200); //wait for 200[msec], Power on Reset

  Wire.begin();          // join i2c bus (address optional for master)
  Wire.setClock(200000); // 200Khz (200Kbps)
  Serial.begin(115200);  // start serial for output (Speed)
  //put your setup code here, to run once:

  pinMode(RESET_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  
  Init_ANMG15(); //Initialize ANMG15
  delay(200); //wait for 100[msec]
    
}
void loop() {

   byte read_data[4] = {0};   //Touch output read data
   byte checksum[2]= {0};     //Checksum Data

   Wire.beginTransmission(ANMG15_ID_VDD); // sned ic slave address
   Wire.write(byte(Output1));             // sends register address
   Wire.endTransmission();                // stop transmitting
   Wire.requestFrom(ANMG15_ID_VDD, 4);    // key data read (2 byte + checksum 2byte)
   while ( Wire.available() )
    {
      read_data[0] = Wire.read();
      read_data[1] = Wire.read();
      read_data[2] = Wire.read(); //Checksum Data
      read_data[3] = Wire.read(); //Checksum Data
   }
   Wire.endTransmission(); // I2C Stop
   
   Serial.write(10);
   Serial.print("-------Touch Sensor Output Data  ---- > "); // Test Code
   delay(20);

   print2hex(read_data, 4);
   Serial.write(LF);
   Serial.write(CR);
     
   checksum[0] = (read_data[0] ^ 0x5A); //Checksum Factor = 0xA5 (Register Address : 0x94h)
   checksum[1] = (read_data[1] ^ 0x5A); //Checksum Factor = 0xA5 (Register Address : 0x94h)
   Serial.print("------- Checksum Verify  ---- > "); // Test Code
   Serial.print( checksum[0],HEX);
   Serial.print( checksum[1],HEX);
   //Serial.write(SP);
   Serial.write(LF);
   Serial.write(CR);

   delay(200);   
}

void  Init_ANMG15(void)
{
   // ---------------- Register unlock Control Start----------------------------

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Unlock_op_en));     // 0x64h
   Wire.write(0x80);
   // 0x80 : Register unlock/unlock 
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Reg_unlock));       // 0x65h
   Wire.write(0xAA);
   // 0xAA : Register Unlock Enable (Address 0x00h ~ 0x61h)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Reg_unlock));       // 0x65h
   Wire.write(0x55);
   // 0x55 : Register Unlock Enable (Address 0x00h ~ 0x61h)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Dread_unlock));     // 0x66h
   Wire.write(0xAA);
   // 0xAA : Register Unlock Enable (Address 0x62h, 0x63h)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Dread_unlock));     // 0x66h
   Wire.write(0x55);
   // 0x55 : Register unlock Enable (Address 0x62h, 0x63h)
   Wire.endTransmission(); //

// --------------- BIST Register Unlock ---------------------------------
   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(BIST_unlock));     // 0xA9h
   Wire.write(0xAA);
   // 0xAA :  Register unlock Enable (Address 0x67h~0xEAh)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(BIST_unlock));     // 0xA9h
   Wire.write(0x55);
   // 0x55 :  Register unlock Enable (Address 0x67h~0xEAh)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Unlock_op_en));     // 0x64h
   Wire.write(0x01);
   // 0x01 : Register unlock/unlock
   Wire.endTransmission(); //

// ---------------- Register unlock Control End----------------------------
   
   //------------------ Software Reset Enable (Set)----------------------
   Wire.beginTransmission(ANMG15_ID_VDD); // 
   Wire.write(byte(Global_ctrl1)); //  0x36h
   Wire.write(byte(0x49)); // Software Reset Enable, Response time ON =3, OFF=3
   Wire.endTransmission(); //

   // --------------- Hidden Register Start ---------------------------------
   // user does not change the register. please contact to us
   // -----------------------------------------------------------------------
 
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x08)); // address
   Wire.write(byte(0x11)); //0x08h
   Wire.write(byte(0x11)); //0x09h
   Wire.write(byte(0x11)); //0x0Ah
   Wire.write(byte(0x11)); //0x0Bh
   Wire.write(byte(0x11)); //0x0Ch
   Wire.write(byte(0x11)); //0x0Dh
   Wire.write(byte(0xA1)); //0x0Eh
   Wire.write(byte(0x10)); //0x0Fh   
   Wire.endTransmission(); //   

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x10)); // address
   Wire.write(byte(0xCE)); //0x10h // IC Reset Value: 0xF5 (-0.55%) ->0xCE (-2.55%)
   Wire.write(byte(0x20)); //0x11h
   Wire.write(byte(0xFF)); //0x12h
//Wire.write(byte(0x88));  //0x12h
//Change the value of the register 0x12h when a problem occurs due to voltage drop
   
   Wire.write(byte(0x92)); //0x13h, CS_Filter1

   Wire.write(byte(0x86)); //0x14h
//IC Reset Value: 0x86 (6ch) , -% Hold Channel 6ch
// Set when there is a possibility of touching multiple touch pads due to the narrow spacing of the touch keypad.
   Wire.write(byte(0x76)); //0x15h
   Wire.write(byte(0x64)); //0x16h
   Wire.write(byte(0xFF)); //0x17h 

   Wire.write(byte(0x1B)); //0x18h ,CS_Filter
// 0x18h IC Reset Value: 0x2B ->0x1B (CS Noise Filter)

   Wire.write(byte(0x11)); //0x19h
   Wire.write(byte(0x03)); //0x1Ah 
   // 0x1Ah IC Reset Value: 0x00 ->0x03 (Init Fast Cal., Up Fast, Down Slow)
   Wire.write(byte(0xFF)); //0x1Bh  
   Wire.write(byte(0x40)); //0x1Ch 
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x21)); // address 
   Wire.write(byte(0x00)); // data
   Wire.endTransmission(); // 

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x35)); // address
   Wire.write(byte(0x40)); // High  Frequency Band , -% Calibration Speed Fast (bit6~bit0)
   //Wire.write(byte(0xC0)); // Default Value (Low Frequency Band)
   //Wire.write(byte(0x40)); // High  Frequency Band
   //IC Reset Value = 0xC0 (Sensing Frequency Low) -> 0x40 (Sensing Frequency High)
   //Change the value to improve the Low-Frequency-Noise.
   Wire.endTransmission(); //   

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x47)); // address ,Sensotovoty Loop
   Wire.write(byte(0x0D)); // data , 
   Wire.endTransmission(); //   
      
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x49)); // address
   Wire.write(byte(0x1C)); // data
   Wire.endTransmission(); //    

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x4A)); // address
   Wire.write(byte(0x04)); // data
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(0x59));// 1st address 0x59h (0x59h~ 0x63h)
   Wire.write(byte(0x00)); //0x59h
   Wire.write(byte(0x00)); //0x5Ah
   Wire.write(byte(0x00)); //0x5Bh
   Wire.write(byte(0x00)); //0x5Ch
   Wire.write(byte(0x00)); //0x5Dh
   Wire.write(byte(0x02)); //0x5Eh
   Wire.write(byte(0x08)); //0x5Fh
   Wire.write(byte(0x02)); //0x60h
   Wire.write(byte(0x08)); //0x61h
   Wire.write(byte(0x00)); //0x62h
   Wire.write(byte(0x00)); //0x63h
   Wire.endTransmission(); // stop

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(0x67)); // address
   Wire.write(byte(0x01)); // data (SEN_INIT_OPTION bit=1)
   Wire.endTransmission(); //

   // --------------- Hidden Register End-------------------------------

   // ---------------- user code ---------------------//   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Ch_enable1)); // 0x01h
   Wire.write(0xFF); // 0:Touch Key Disable, 1: Touch Key Enable
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Ch_enable2)); // 0x01h
   Wire.write(0x7F); // 0:Touch Key Disable, 1: Touch Key Enable
   Wire.endTransmission(); //

// ------------ Calibration Speed Control ------------------------ 
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Cal_speed)); // 0x30h
   Wire.write(0x67); // Down Calibration Speed = Normal -> Slow
   //Wire.write(0x66); // Default Value 
   Wire.endTransmission(); //   
   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Cal_BS_speed)); // 0x31h
   Wire.write(0x67); // Down Calibration Speed = Normal -> Slow
   //Wire.write(0x66); // Default Value
   Wire.endTransmission(); //   

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Global_ctrl0)); //  0x32h
   Wire.write(0x40); //OUT Pin Option(Default= CS1 Out, Active Low, Open-drain)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Global_ctrl2)); // 0x33h
   Wire.write(0xBC); // Expire time Max set (0b1111)
   // Referencr to  Register Address 0x68h, 0x69h
   //Wire.write(0x80); // Expire time disable
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Clock_ctrl)); //  0x34h
   Wire.write(0x65); // //IC Reset value 0x05 -> 0x65
   Wire.endTransmission(); // 

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(State_count)); // 0x37
   Wire.write(0xE6); // //IC Reset value 0xFF -> 0xE6 
   // Calibration Speed Pre-scaler , (bit4 ~ bit0)
   //The value can be changed according to the operating conditions of the set.
   Wire.endTransmission(); //   
   
   
//------------ Sensitivity control  -----------------------------------
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity1)); // 0x38h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //      

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity2)); // 0x39h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //   
   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity3)); // 0x3Ah
   Wire.write(0x28); // HEX x 0.025 =1.0%
   Wire.endTransmission(); //   
   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity4)); // 0x3Bh
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //   
   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity5)); // 0x3Ch
   Wire.write(0x28); // HEX x 0.025 =1.0%
   Wire.endTransmission(); //   
   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity6)); // 0x3Dh
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //   
   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity7)); // 0x3Eh
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //   
   
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity8)); // 0x3Fh
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity9)); // 0x40h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity10)); // 0x41h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity11)); // 0x42h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity12)); // 0x43h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity13)); // 0x44h
   Wire.write(0x28); // HEX x 0.025 =1.0%
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity14)); // 0x45h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //  

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitivity15)); // 0x46h
   Wire.write(0x28); // HEX x 0.025 = 1.0%
   Wire.endTransmission(); //  

//----------- Interrupt Option -----------------------------------------
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Init_ctrl)); // 0x4Ch
   Wire.write(0x00); // Interrupt Option(Default; levle Mode, 5ms,when touch Output Changes)
   Wire.endTransmission(); //  

 //-------- Hysteresis Control Register -------------------------------------------------
   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Global_ctrl0)); // address 0x05h
   Wire.write(byte(0xC0)); // HW Reset Value=0x80, Hysteresis Enable/Disable bit=1(Enable) 
   Wire.endTransmission(); //   

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sensitiviy_tune)); // address 0x48h
   Wire.write(byte(0x48)); // sign=1(-1), hys_level=-8 (0.025% x -8 = - 0.2%)
   Wire.endTransmission(); //   

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sys_ctrl1)); // address 0xD1h
   Wire.write(byte(0x3C)); // Hys_level_ropt(bit6~bit4) = 011:12.5%(Default)
   Wire.endTransmission(); // 1.0% x 12.5% = 0.875%

   Wire.beginTransmission(ANMG15_ID_VDD);// 
   Wire.write(byte(Sys_ctrl2)); // address 0xD2h
   Wire.write(byte(0x34)); // Hys_level_opt=1: Select ratio hysteresis level
   Wire.endTransmission(); //   

// ---------------- Output Expiration Contrl Register --------------------------
   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Texp_ctrl1));       // 0x68h
   Wire.write(0x07); // System freq. 8sec
   // 0x01 : Resolution Option for touch expiration time  00-1sec, 1-> time base system frepuency
   // Time set : Register address 0x33h -> Output expiration Time control bit
   // Time = 0x33h : 1111 x 0x68h Value : Resolution 8sec = 8sec x 1111
   // Wire.write(0x07);  // -> 0x33h Time 0b1111 x 8sec Resolution = 120sec
   Wire.endTransmission();             //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Texp_ctrl2));       // 0x69h
   Wire.write(0xFF);                   
   // 0xFF : To channel seletion for the touch expiration function 
   // Wire.write(0x01); // if 0x01 = CS1 , 0xFF = CS8 ~ CS1
   Wire.endTransmission();             //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Texp_ctrl3));       // 0x6Ah
   Wire.write(0x7F);                   
   // 0xFF : To channel seletion for the touch expiration function 
   // Wire.write(0x01); // if 0x01 = CS9 , 0xFF = CS15 ~ CS9
   Wire.endTransmission();             //

   // ---------------- DTR (Direct touch reset) Contrl Register --------------------------
   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(DTR_ctrl1));        // 0x6Bh
   //Wire.write(0x4E);                 // 0x4E : Set the Touch Nagative Percent Limit. -2.4%
   Wire.write(0x27);                   // 0x27 : Set the Touch Nagative Percent Limit. -1.2%
   Wire.endTransmission();             //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(DTR_ctrl2));           // 0x6Ch
   Wire.write(0x00);                      // 0x00
   // Set the Touch -% Percent Limit.
   // (( 0x6Ch Value *256 + 0x6Bh Value) * 0.4) / (0x47h Value * -1) = Nagative %
   // Ex, 0x6Ch = 0x00, 0x6B = 0x4E, 0x47h = 0x0D, ((0 *256 + 78)*0.4) / (13 * -1) = -2.4%
   // Ex, 0x6Ch = 0x00, 0x6B = 0x27, 0x47h = 0x0D, ((0 *256 + 39)*0.4) / (13 * -1) = -1.2%
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(DTR_ctrl3));        // 0x6Dh
   Wire.write(0xF1); // 
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(DTR_ictrl));        // 0xD9h
   //Wire.write(0x04); // DTR_Period (Sensing Cycle), 0x04 = 4 Sensing-Cycle
   Wire.write(0x3F); // DTR_Period (Sensing Cycle), 0x3F = 64 Sensing-Cycle
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(DTR_ictrl1));        // 0xDAh ~ 0xE8
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS1
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS2
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS3
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS4
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS5
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS6
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS7   
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS8
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS9
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS10
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS11
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS12
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS13
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS14
   Wire.write(0x8C); // Set the Touch DTR Percent Limit CS15
   Wire.endTransmission(); //

//-------- Checksum Control -----------------------------------
   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Checksum_ctrl));        // 0x6Dh
   Wire.write(0x04); // Csum_fopt=01, Checksum factor = 0x5A
   Wire.endTransmission(); //


   //------------------ Software Reset Disable (Clear) ---------------------
   Wire.beginTransmission(ANMG15_ID_VDD); // 
   Wire.write(byte(Global_ctrl1)); //  0x36h
   Wire.write(byte(0x48)); // Software Reset Disable, Response time ON =3, OFF=3
   Wire.endTransmission(); //

   // ---------------- Register Lock Control ---------------------------------
   // It is recommended to use it to prepare for communication errors. 
   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Unlock_op_en));     // 0x64h
   Wire.write(0x80);                   
   // 0x80 : Register Lock Enable (Address 0x00h ~ 0x63h)
   Wire.endTransmission();             //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Reg_unlock));       // 0x65h
   Wire.write(0x01);                   
   // 0x01 : Register Lock Enable (Address 0x00h ~ 0x61h)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(Dread_unlock));     // 0x66h
   Wire.write(0x01);                   
   // 0x01 : Register Lock Enable (Address 0x62h, 0x63h)
   Wire.endTransmission(); //

   Wire.beginTransmission(ANMG15_ID_VDD); //
   Wire.write(byte(BIST_unlock));      // 0xA9h
   Wire.write(0x01);                   
   // 0x01 : BIST Register Lock Enable (Address 0x67h ~ 0x77h)
   Wire.endTransmission(); //

   //================ END Initialize ANMG15 ==================================
   }

// End


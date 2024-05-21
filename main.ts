/*
Riven
modified from pxt-servo/servodriver.ts
load dependency
"robotbit": "file:../pxt-robotbit"
*/
const enum DistanceUnit {
    //% block="cm"
    CM = 58, // Duration of echo round-trip in Microseconds (uS) for two centimeters, 343 m/s at sea level and 20°C
    //% block="inch"
    INCH = 148, // Duration of echo round-trip in Microseconds (uS) for two inches, 343 m/s at sea level and 20°C
  }

//% color="#63AEE6" weight=10 icon="\uf19d"
namespace Robobox {
    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    // HT16K33 commands
    const HT16K33_ADDRESS = 0x70
    const HT16K33_BLINK_CMD = 0x80
    const HT16K33_BLINK_DISPLAYON = 0x01
    const HT16K33_BLINK_OFF = 0
    const HT16K33_BLINK_2HZ = 1
    const HT16K33_BLINK_1HZ = 2
    const HT16K33_BLINK_HALFHZ = 3
    const HT16K33_CMD_BRIGHTNESS = 0xE0

    export enum Servos {
        S1 = 0x01,
        S2 = 0x02,
        S3 = 0x03,
        S4 = 0x04,
        S5 = 0x05,
        S6 = 0x06,
        S7 = 0x07,
        S8 = 0x08
    }

    export enum Motors {
        M1A = 0x1,
        M1B = 0x2,
        M2A = 0x3,
        M2B = 0x4
    }

    export enum Steppers {
        M1 = 0x1,
        M2 = 0x2
    }

    export enum SonarVersion {
        V1 = 0x1,
        V2 = 0x2
    }

    export enum Turns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }

    export enum Stift_Bewegung {
        auf = 0,
        ab = 1
    }

     export enum vor_zurueck_bewegung {
        vor = 0,
        zurueck = 1
    }
      export enum links_rechts_bewegung {
        links = 0,
        rechts = 1
    }
    

    let initialized = false
    let initializedMatrix = false
    let neoStrip: neopixel.Strip;
    let matBuf = pins.createBuffer(17);
  let distanceBuf = 0;
     let speed_custom_l = 255;
     let speed_custom_r = 255;
  let trim_l = 0;
    let trim_r = 0;
    let trim_l_a = 0;
    let trim_r_a = 0;

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        //serial.writeValue("ch", channel)
        //serial.writeValue("on", on)
        //serial.writeValue("off", off)

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }


    function setStepper(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(0, STP_CHA_L, STP_CHA_H);
                setPwm(2, STP_CHB_L, STP_CHB_H);
                setPwm(1, STP_CHC_L, STP_CHC_H);
                setPwm(3, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(3, STP_CHA_L, STP_CHA_H);
                setPwm(1, STP_CHB_L, STP_CHB_H);
                setPwm(2, STP_CHC_L, STP_CHC_H);
                setPwm(0, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(4, STP_CHA_L, STP_CHA_H);
                setPwm(6, STP_CHB_L, STP_CHB_H);
                setPwm(5, STP_CHC_L, STP_CHC_H);
                setPwm(7, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(7, STP_CHA_L, STP_CHA_H);
                setPwm(5, STP_CHB_L, STP_CHB_H);
                setPwm(6, STP_CHC_L, STP_CHC_H);
                setPwm(4, STP_CHD_L, STP_CHD_H);
            }
        }
    }

    function stopMotor(index: number) {
        setPwm((index - 1) * 2, 0, 0);
        setPwm((index - 1) * 2 + 1, 0, 0);
    }

    /**
     * Init RGB pixels mounted on robotbit
     */
    //% blockId="robotbit_rgb" block="RGB"
    //% weight=70
    export function rgb(): neopixel.Strip {
        if (!neoStrip) {
            neoStrip = neopixel.create(DigitalPin.P16, 4, NeoPixelMode.RGB)
        }

        return neoStrip;
  }
  
  /*#############################################################################Fahren Basic START###############################################*/
   /**
   * @param speed_c value of the speed between 1 and 100. eg: 100
//% blockId=setSpeed_custom block="Geschwindigkeit %speed_c|%"
  //% speed_c.min=1 speed_c.max=100
  //% group="Setup" weight=1
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Basic" weight=90
  export function setCustomSpeed(speed_c: number): void {
    speed_custom_l = Math.map(speed_c, 1, 100, 90, 255)
    speed_custom_r = Math.map(speed_c, 1, 100, 90, 255)
  } */
     /**
   * @param trim_l_block value of the speed between 1 and 10. eg: 10 */
//% blockId=trim_l_block block="Trimmen rechts %trim_l_block|%"
  //% trim_l_block.min=0 trim_l_block.max=100
  //% group="Setup" weight=1
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Basic" weight=90
  export function settriml(trim_l_block: number): void {
    trim_l = trim_l_block;
  }

    /**
   * @param trim_r_block value of the speed between 1 and 10. eg: 10 */
//% blockId=trim_r_block block="Trimmen links %trim_r_block|%"
  //% trim_r_block.min=0 trim_r_block.max=100
  //% group="Setup" weight=1
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Basic" weight=90
  export function settrimr(trim_r_block: number): void {
    trim_r = trim_r_block;
  }
 
    //% blockId=robotbit_Beebot_vor block="Fahren %vorzur_index |Dauer %delay|ms"
    //% group="Linea/Manuva" weight=6
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Fahren Basic" weight=90
    //export function BeeVor(delay: number): void {
     export function BeeVor(vorzur_index: vor_zurueck_bewegung, delay: number ): void {
      if(vorzur_index == 0) {
      MotorRun(0x1, speed_custom_r-trim_l);
      MotorRun(0x2, speed_custom_r-trim_l);
      MotorRun(0x3, speed_custom_l-trim_r);
      MotorRun(0x4, speed_custom_l-trim_r);
      basic.pause(delay);
      MotorStopAll() }
         else {
     MotorRun(0x1, -speed_custom_r+trim_l);
      MotorRun(0x2, -speed_custom_r+trim_l);
      MotorRun(0x3, -speed_custom_l+trim_r);
      MotorRun(0x4, -speed_custom_l+trim_r);
      basic.pause(delay);
      MotorStopAll()        
         }
         
  }
    
  //% blockId=robotbit_Beebot_klinks block="Fahren Kurve %links_rechts_index |Dauer %delay|ms"
    //% group="Linea/Manuva" weight=4
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Fahren Basic" weight=90
  //   export function BeeKurveLinks(links_rechts_index: links_rechts_bewegung, delay: number ): void {
  //     if(links_rechts_index == 0) {
  //     MotorRun(0x1, speed_custom_r-trim_l);
  //     MotorRun(0x2, speed_custom_r-trim_l);
  //     basic.pause(delay);
  //     MotorStopAll()
  //     } else {
  //     MotorRun(0x3, speed_custom_l-trim_r);
  //     MotorRun(0x4, speed_custom_l-trim_r);
  //     basic.pause(delay);
  //     MotorStopAll()
  //     }
  // }
    
   //% blockId=robotbit_Beebot_rechts block="Drehen %links_rechts_index |Dauer %delay|ms"
  //% group="Linea/Manuva" weight=3
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Basic" weight=90
  export function BeeRechts(links_rechts_index: links_rechts_bewegung, delay: number ): void {
    if(links_rechts_index == 0) {
    MotorRun(0x1, speed_custom_r-trim_l);
    MotorRun(0x2, speed_custom_r-trim_l);
    MotorRun(0x3, -speed_custom_l+trim_r);
    MotorRun(0x4, -speed_custom_l+trim_r);
    basic.pause(delay);
    MotorStopAll()
    } else {
    MotorRun(0x1, -speed_custom_r+trim_l);
    MotorRun(0x2, -speed_custom_r+trim_l);
    MotorRun(0x3, speed_custom_l-trim_r);
    MotorRun(0x4, speed_custom_l-trim_r);
    basic.pause(delay);
    MotorStopAll() 
        
    }
  }

  
   //% blockId=robotbit_Manuva_linksvordiag block="Diagonal %links_rechts_index Vor |Dauer %delay|ms"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Fahren Basic" weight=110
  //   export function Manuvalvd(links_rechts_index: links_rechts_bewegung, delay: number ): void {
  //     if(links_rechts_index != 0) {
  //       MotorRun(0x1, speed_custom_r-trim_l);
  //     MotorRun(0x3, speed_custom_l-trim_r);
  //     basic.pause(delay);
  //     MotorStopAll()
  //     } else {
  //          MotorRun(0x2, speed_custom_r-trim_l);
  //     MotorRun(0x4, speed_custom_l-trim_r);
  //     basic.pause(delay);
  //     MotorStopAll()
  //     }
  // }
   
   //% blockId=robotbit_Manuva_linksrueckdiag block="Diagonal %links_rechts_index Zurück |Dauer %delay|ms"
  //% group="Manuva" weight=110
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Basic" weight=110
  //   export function Manuvalrd(links_rechts_index: links_rechts_bewegung, delay: number ): void {
  //       if(links_rechts_index != 0) {
  //     MotorRun(0x2, -speed_custom_r+trim_l);
  //     MotorRun(0x4, -speed_custom_l+trim_r);
  //     basic.pause(delay);
  //     MotorStopAll()
  //       } else {
  //     MotorRun(0x1, -speed_custom_r+trim_l);
  //     MotorRun(0x3, -speed_custom_l+trim_r);
  //     basic.pause(delay);
  //     MotorStopAll()
  //       }
  // }
   
   //% blockId=robotbit_Manuva_rechtsschieben block="Fahren %links_rechts_index |Dauer %delay|ms"
  //% group="Manuva" weight=110
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Basic" weight=110

//  export function Manuvarrs(links_rechts_index: links_rechts_bewegung, delay: number ): void {
//       if(links_rechts_index != 0) {
//     MotorRun(0x1, speed_custom_r-trim_l);
//     MotorRun(0x2, -speed_custom_r+trim_l);
//     MotorRun(0x3, speed_custom_l-trim_r);
//     MotorRun(0x4, -speed_custom_l+trim_r);
//     basic.pause(delay);
//     MotorStopAll()
//       }
//      else {
         
//     MotorRun(0x1, -speed_custom_r+trim_l);
//     MotorRun(0x2, speed_custom_r-trim_l);
//     MotorRun(0x3, -speed_custom_l+trim_r);
//     MotorRun(0x4, speed_custom_l-trim_r);
//     basic.pause(delay);
//     MotorStopAll()
//      }
//   }
   
 
  
  /*#############################################################################Fahren Basic ENDE###############################################*/
      /*#############################################################################Fahren Advanced START###############################################*/
   /**
   * @param speed_c value of the speed between 1 and 100. eg: 100 
//% blockId=setSpeed_custom_adv block="Geschwindigkeit %speed_c|%"
  //% speed_c.min=1 speed_c.max=100
  //% group="Setup" weight=1
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Advanced" weight=90
  export function setCustomSpeed_a(speed_c: number = 100): void {
    speed_custom_l = Math.map(speed_c, 1, 100, 90, 255)
    speed_custom_r = Math.map(speed_c, 1, 100, 90, 255)
  }*/
     /**
   * @param trim_l_block_a value of the speed between 1 and 10. eg: 10 */
//% blockId=trim_l_block_adv block="Trimmen rechts %trim_l_block_a|%"
  //% trim_l_block_a.min=0 trim_l_block_a.max=100
  //% group="Setup" weight=1
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Advanced" weight=90
  export function settriml_a(trim_l_block_a: number = 1): void {
    trim_l_a = trim_l_block_a;
  }

    /**
   * @param trim_r_block_a value of the speed between 1 and 10. eg: 10 */
//% blockId=trim_r_block_adv block="Trimmen links %trim_r_block_a|%"
  //% trim_r_block_a.min=0 trim_r_block_a.max=100
  //% group="Setup" weight=1
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Advanced" weight=90
  export function settrimr_a(trim_r_block_a: number = 1): void {
    trim_r_a = trim_r_block_a;
  }
 
     //% blockId=robotbit_Beebot_vor_adv block="Fahren %vorzur_index"
    //% group="Linea/Manuva" weight=6
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Fahren Advanced" weight=90
    //export function BeeVor(delay: number): void {
     export function BeeVor_adv(vorzur_index: vor_zurueck_bewegung): void {
      if(vorzur_index == 0) {
      MotorRun(0x1, speed_custom_r-trim_l_a);
      MotorRun(0x2, speed_custom_r-trim_l_a);
      MotorRun(0x3, speed_custom_l-trim_r_a);
      MotorRun(0x4, speed_custom_l-trim_r_a);
 }
         else {
     MotorRun(0x1, -speed_custom_r+trim_l_a);
      MotorRun(0x2, -speed_custom_r+trim_l_a);
      MotorRun(0x3, -speed_custom_l+trim_r_a);
      MotorRun(0x4, -speed_custom_l+trim_r_a);
       
         }
         
  }
  //% blockId=robotbit_Beebot_klinks_adv block="Fahren Kurve %links_rechts_index"
    //% group="Linea/Manuva" weight=4
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Fahren Advanced" weight=90
    export function BeeKurveLinks_a(links_rechts_index: links_rechts_bewegung): void {
      if(links_rechts_index == 0) {
      MotorRun(0x1, speed_custom_r-trim_l_a);
      MotorRun(0x2, speed_custom_r-trim_l_a);
      } else {
      MotorRun(0x3, speed_custom_l-trim_r_a);
      MotorRun(0x4, speed_custom_l-trim_r_a);
      }

  }
    
   //% blockId=robotbit_Beebot_rechts_adv block="Drehen %links_rechts_index"
  //% group="Linea/Manuva" weight=3
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Advanced" weight=90
  export function BeeRechts_a(links_rechts_index: links_rechts_bewegung): void {
   if(links_rechts_index == 0) {
    MotorRun(0x1, speed_custom_r-trim_l_a);
    MotorRun(0x2, speed_custom_r-trim_l_a);
    MotorRun(0x3, -speed_custom_l+trim_r_a);
    MotorRun(0x4, -speed_custom_l+trim_r_a);
    } else {
    MotorRun(0x1, -speed_custom_r+trim_l_a);
    MotorRun(0x2, -speed_custom_r+trim_l_a);
    MotorRun(0x3, speed_custom_l-trim_r_a);
    MotorRun(0x4, speed_custom_l-trim_r_a);
    }

  }

   //% blockId=robotbit_Manuva_linksvordiag_adv block="Diagonal %links_rechts_index Vor"
    //% group="Manuva" weight=110
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory="Fahren Advanced" weight=110
    export function Manuvalvd_a(links_rechts_index: links_rechts_bewegung): void {
      if(links_rechts_index == 0) {
        MotorRun(0x1, speed_custom_r-trim_l_a);
      MotorRun(0x3, speed_custom_l-trim_r_a);
      } else {
           MotorRun(0x2, speed_custom_r-trim_l_a);
      MotorRun(0x4, speed_custom_l-trim_r_a);
      }
  }
   
   //% blockId=robotbit_Manuva_linksrueckdiag_adv block="Diagonal %links_rechts_index Zurück"
  //% group="Manuva" weight=110
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Advanced" weight=110
  export function Manuvalrd_a(links_rechts_index: links_rechts_bewegung): void {
     if(links_rechts_index == 0) {
      MotorRun(0x2, -speed_custom_r+trim_l_a);
      MotorRun(0x4, -speed_custom_l+trim_r_a);
        } else {
      MotorRun(0x1, -speed_custom_r+trim_l_a);
      MotorRun(0x3, -speed_custom_l+trim_r_a);
        }
  }
   
   //% blockId=robotbit_Manuva_rechtsschieben_adv block="Fahren %links_rechts_index"
  //% group="Manuva" weight=110
  //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
  //% subcategory="Fahren Advanced" weight=110
  export function Manuvarrs_a(links_rechts_index: links_rechts_bewegung): void {
    if(links_rechts_index == 0) {
    MotorRun(0x1, speed_custom_r-trim_l_a);
    MotorRun(0x2, -speed_custom_r+trim_l_a);
    MotorRun(0x3, speed_custom_l-trim_r_a);
    MotorRun(0x4, -speed_custom_l+trim_r_a);
      }
     else {  
    MotorRun(0x1, -speed_custom_r+trim_l_a);
    MotorRun(0x2, speed_custom_r-trim_l_a);
    MotorRun(0x3, -speed_custom_l+trim_r_a);
    MotorRun(0x4, speed_custom_l-trim_r_a);
     }
  }
  
    //% blockId=robotbit_stop_all_adv block="Alle Motoren stoppen"
  //% group="Motoren stoppen" weight=1
  //% blockGap=50
  //% subcategory="Fahren Advanced" weight=95
  export function MotorStopAll_adv(): void {
      if (!initialized) {
          initPCA9685()
      }
      for (let idx = 1; idx <= 4; idx++) {
          stopMotor(idx);
      }
  }
 
  
  /*#############################################################################Fahren Advanced ENDE###############################################*/


  /*#############################################################################Fahren Expert Anfang###############################################*/

  
    //% blockId=robotbit_motor_run block="Motor|%index|Geschwindigkeit %speed"
    //% group="Motor" weight=120
    //% speed.min=-100 speed.max=100
    //% subcategory="Fahren Expert (Experimentiell)" weight=95
    export function MotorRun(index: Motors, speed: number): void {
      if (!initialized) {
          initPCA9685()
      }
      speed = speed * 16; // map 255 to 4096
      if (speed >= 4096) {
          speed = 4095
      }
      if (speed <= -4096) {
          speed = -4095
      }
      if (index > 4 || index <= 0)
          return
      let pp = (index - 1) * 2
      let pn = (index - 1) * 2 + 1
      if (speed >= 0) {

          setPwm(pp, 0, Math.map(speed, 0, 100, 0, 255))
          setPwm(pn, 0, 0)
      } else {
          setPwm(pp, 0, 0)
          setPwm(pn, 0, -Math.map(speed, 0, 100, 0, 255))
      }
  }


  /**
   * Execute two motors at the same time
   * @param motor1 First Motor; eg: M1A, M1B
   * @param speed1 [-255-255] speed of motor; eg: 100, -100
   * @param motor2 Second Motor; eg: M2A, M2B
   * @param speed2 [-255-255] speed of motor; eg: 100, -100
  */
  //% blockId=robotbit_motor_dual block="Motor|%motor1|Geschwindigkeit %speed1|%motor2|Geschwindigkeit %speed2"
  //% group="Motor" weight=120
  //% speed1.min=-100 speed1.max=100
  //% speed2.min=-100 speed2.max=100
  //% subcategory="Fahren Expert (Experimentiell)" weight=95
  
  export function MotorRunDual(motor1: Motors, speed1: number, motor2: Motors, speed2: number): void {
      MotorRun(motor1, speed1);
      MotorRun(motor2, speed2);
  }

  /**
   * Execute single motors with delay
   * @param index Motor Index; eg: M1A, M1B, M2A, M2B
   * @param speed [-255-255] speed of motor; eg: 100, -100
   * @param delay seconde delay to stop; eg: 1
  */
  //% blockId=robotbit_motor_rundelay block="Motor|%index|Geschwindigkeit %speed|Dauer %delay|Millisekunden"
  //% group="Motor" weight=120
  //%blockGap=8
  //% speed.min=-0 speed.max=100
  //% subcategory="Fahren Expert (Experimentiell)" weight=95
  export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
      MotorRun(index, speed);
      basic.pause(delay); 
      MotorRun(index, 0);
  }



  //% blockId=robotbit_stop block="Motor stoppen|%index|"
  //% group="Motor" weight=120
  //% subcategory="Fahren Expert (Experimentiell)" weight=95
  export function MotorStop(index: Motors): void {
      MotorRun(index, 0);
  }

  //% blockId=robotbit_stop_all block="Alle Motoren stoppen"
  //% group="Motor" weight=120
  //% blockGap=50
  //% subcategory="Fahren Expert (Experimentiell)" weight=95
  export function MotorStopAll(): void {
      if (!initialized) {
          initPCA9685()
      }
      for (let idx = 1; idx <= 4; idx++) {
          stopMotor(idx);
      }
  }
  /*#############################################################################Fahren Expert ENDE###############################################*/

  /*#############################################################################Stift ANFANG###############################################*/
/**
     * Stift Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=robotbit_stift_rauf_adv block="Stift %aufab | %index"
    //% group="Linea" weight=100
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
     //% subcategory="Stift" weight=100
     export function Stift_rauf_adv(index: Servos, stift_index: Stift_Bewegung): void {
    let winkel = 50;
      if(stift_index == 0) {
           winkel = 50;
         } 
          else if (stift_index == 1) {
           winkel = 90;
            }
         
      if (!initialized) {
          initPCA9685()
      }
      // 50hz: 20,000 us
      let v_us = (winkel * 1800 / 180 + 600) // 0.6 ~ 2.4
      let value = v_us * 4096 / 20000
      setPwm(index + 7, 0, value)
  }
  
  /*#############################################################################Stift ENDE###############################################*/
  
      
 /*#############################################################################Roboterarm Anfang###############################################*/      
    
    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=robotbit_servo block="Servo|%index|Grad %degree"
    //% group="Servo" weight=90
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
     //% subcategory="RoboterArm" weight=90
     export function Servo(index: Servos, degree: number): void {
      if (!initialized) {
          initPCA9685()
      }
      // 50hz: 20,000 us
      let v_us = (degree * 1800 / 180 + 600) // 0.6 ~ 2.4
      let value = v_us * 4096 / 20000
      setPwm(index + 7, 0, value)
  }


  //% blockId=robotbit_stepper_degree block="Schrittmotor 28BYJ-48|%index|Grad %degree"
  //% group="Schrittmotor" weight=90
  //% subcategory="RoboterArm" weight=90
  export function StepperDegree(index: Steppers, degree: number): void {
      if (!initialized) {
          initPCA9685()
      }
      setStepper(index, degree > 0);
      degree = Math.abs(degree);
      basic.pause(10240 * degree / 360);
      MotorStopAll()
  }


  //% blockId=robotbit_stepper_turn block="Schrittmotor 28BYJ-48|%index|drehe %turn"
  //% group="Schrittmotor" weight=90
  //% subcategory="RoboterArm" weight=90
  export function StepperTurn(index: Steppers, turn: Turns): void {
      let degree = turn;
      StepperDegree(index, degree);
  }

  /*#############################################################################Roboterarm Ende###############################################*/
  
  
 /*#############################################################################Ultraschall Anfang###############################################*/
    
const MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID = 798;
const MAX_ULTRASONIC_TRAVEL_TIME = 300 * DistanceUnit.CM;
const ULTRASONIC_MEASUREMENTS = 3;

interface UltrasonicRoundTrip {
  ts: number;
  rtt: number;
}

interface UltrasonicDevice {
  trig: DigitalPin | undefined;
  roundTrips: UltrasonicRoundTrip[];
  medianRoundTrip: number;
  travelTimeObservers: number[];
}

let ultrasonicState: UltrasonicDevice;

/**
 * Configures the ultrasonic distance sensor and measures continuously in the background.
 * @param trig pin connected to trig, eg: DigitalPin.P2
 * @param echo pin connected to echo, eg: DigitalPin.P8
 */
//% subcategory="Abstandssensor"
//% blockId="makerbit_ultrasonic_connect"
//% block="Verbinde Abstandssensor | mit Trig an %trig | und Echo an %echo"
//% trig.fieldEditor="gridpicker"
//% trig.fieldOptions.columns=4
//% trig.fieldOptions.tooltips="false"
//% echo.fieldEditor="gridpicker"
//% echo.fieldOptions.columns=4
//% echo.fieldOptions.tooltips="false"
//% weight=80
export function connectUltrasonicDistanceSensor(
  trig: DigitalPin,
  echo: DigitalPin
): void {
  if (ultrasonicState && ultrasonicState.trig) {
    return;
  }

  if (!ultrasonicState) {
    ultrasonicState = {
      trig: trig,
      roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
      medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
      travelTimeObservers: [],
    };
  } else {
    ultrasonicState.trig = trig;
  }

  pins.onPulsed(echo, PulseValue.High, () => {
    if (
      pins.pulseDuration() < MAX_ULTRASONIC_TRAVEL_TIME &&
      ultrasonicState.roundTrips.length <= ULTRASONIC_MEASUREMENTS
    ) {
      ultrasonicState.roundTrips.push({
        ts: input.runningTime(),
        rtt: pins.pulseDuration(),
      });
    }
  });

  control.inBackground(measureInBackground);
}

/**
 * Do something when an object is detected the first time within a specified range.
 * @param distance distance to object, eg: 20
 * @param unit unit of distance, eg: DistanceUnit.CM
 * @param handler body code to run when the event is raised
 */
//% subcategory="Abstandssensor"
//% blockId=makerbit_ultrasonic_on_object_detected
//% block="Objekt näher als | %distance | %unit"
//% weight=69
export function onUltrasonicObjectDetected(
  distance: number,
  unit: DistanceUnit,
  handler: () => void
) {
  if (distance <= 0) {
    return;
  }

  if (!ultrasonicState) {
    ultrasonicState = {
      trig: undefined,
      roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
      medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
      travelTimeObservers: [],
    };
  }

  const travelTimeThreshold = Math.imul(distance, unit);

  ultrasonicState.travelTimeObservers.push(travelTimeThreshold);

  control.onEvent(
    MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID,
    travelTimeThreshold,
    () => {
      handler();
    }
  );
}

/**
 * Returns the distance to an object in a range from 1 to 300 centimeters or up to 118 inch.
 * The maximum value is returned to indicate when no object was detected.
 * -1 is returned when the device is not connected.
 * @param unit unit of distance, eg: DistanceUnit.CM
 */
//% subcategory="Abstandssensor"
//% blockId="makerbit_ultrasonic_distance"
//% block="Abstand in %unit"
//% weight=60
export function getUltrasonicDistance(unit: DistanceUnit): number {
  if (!ultrasonicState) {
    return -1;
  }
  basic.pause(0); // yield to allow background processing when called in a tight loop
  return Math.idiv(ultrasonicState.medianRoundTrip, unit);
}

/**
 * Returns `true` if an object is within the specified distance. `false` otherwise.
 *
 * @param distance distance to object, eg: 20
 * @param unit unit of distance, eg: DistanceUnit.CM
 */
//% subcategory="Abstandssensor"
//% blockId="makerbit_ultrasonic_less_than"
//% block="Abstand zum Objekt ist kleiner als | %distance | %unit"
//% weight=50
export function isUltrasonicDistanceLessThan(
  distance: number,
  unit: DistanceUnit
): boolean {
  if (!ultrasonicState) {
    return false;
  }
  basic.pause(0); // yield to allow background processing when called in a tight loop
  return Math.idiv(ultrasonicState.medianRoundTrip, unit) < distance;
}

/*#############################################################################Ultraschall ENDE###############################################*/


  
    function triggerPulse() {
      // Reset trigger pin
      pins.setPull(ultrasonicState.trig, PinPullMode.PullNone);
      pins.digitalWritePin(ultrasonicState.trig, 0);
      control.waitMicros(2);
  
      // Trigger pulse
      pins.digitalWritePin(ultrasonicState.trig, 1);
      control.waitMicros(10);
      pins.digitalWritePin(ultrasonicState.trig, 0);
    }
  
    function getMedianRRT(roundTrips: UltrasonicRoundTrip[]) {
      const roundTripTimes = roundTrips.map((urt) => urt.rtt);
      return median(roundTripTimes);
    }
  
    // Returns median value of non-empty input
    function median(values: number[]) {
      values.sort((a, b) => {
        return a - b;
      });
      return values[(values.length - 1) >> 1];
    }
  
    function measureInBackground() {
      const trips = ultrasonicState.roundTrips;
      const TIME_BETWEEN_PULSE_MS = 145;
  
      while (true) {
        const now = input.runningTime();
  
        if (trips[trips.length - 1].ts < now - TIME_BETWEEN_PULSE_MS - 10) {
          ultrasonicState.roundTrips.push({
            ts: now,
            rtt: MAX_ULTRASONIC_TRAVEL_TIME,
          });
        }
  
        while (trips.length > ULTRASONIC_MEASUREMENTS) {
          trips.shift();
        }
  
        ultrasonicState.medianRoundTrip = getMedianRRT(
          ultrasonicState.roundTrips
        );
  
        for (let i = 0; i < ultrasonicState.travelTimeObservers.length; i++) {
          const threshold = ultrasonicState.travelTimeObservers[i];
          if (threshold > 0 && ultrasonicState.medianRoundTrip <= threshold) {
            control.raiseEvent(
              MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID,
              threshold
            );
            // use negative sign to indicate that we notified the event
            ultrasonicState.travelTimeObservers[i] = -threshold;
          } else if (
            threshold < 0 &&
            ultrasonicState.medianRoundTrip > -threshold
          ) {
            // object is outside the detection threshold -> re-activate observer
            ultrasonicState.travelTimeObservers[i] = -threshold;
          }
        }
  
        triggerPulse();
        basic.pause(TIME_BETWEEN_PULSE_MS);
      }
    }

}

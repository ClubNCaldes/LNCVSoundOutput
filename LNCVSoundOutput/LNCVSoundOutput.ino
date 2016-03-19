#include <LocoNet.h>
#include <EEPROM.h>

/******************************************************************************
 *
 *  Copyright (C) 2014 Daniel Guisado - http://www.clubncaldes.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program; if not, If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************
 *
 * DESCRIPTION
 *
 * This is a Demo for a LNCV outputs and sound module.
 *
 ******************************************************************************/
// uncomment this to debug
//#define DEBUG

#define MAJORVERSION 7
#define MINORVERSION 0

#define LOCONET_TX_PIN 7
#define ARTNR 9001
#define LNCV_COUNT 80

#define LNCV_FIRSTSOUND 35
#define LNCV_NUMSOUNDS 36
#define LNCV_MASTERVOL 37
#define LNCV_SOUNDSOURCE 38
#define LNCV_STOPSOUND 39
#define LNCV_PLAYRANDOM 40
#define LNCV_SNDCFG_OFFSET 50

#define LNCV_COMMAND 100

#define MP3_SDCARD 0xA0
#define MP3_SPI    0xA1
#define MP3_UDISK  0xA2
#define MP3_MODESINGLE 0x00
#define MP3_MODEREPEAT 0x01
#define MP3_MODEREPEATALL 0x02
#define MP3_MODERANDOM 0x03

uint16_t lncv[LNCV_COUNT];
lnMsg *LnPacket;
LocoNetCVClass lnCV;

boolean modeProgramming = false;

byte MP3_source;  //Source of sounds SDCARD | SPI | UDISK
unsigned char cmd_buf[10];

/* Pin assignment to each output */
uint8_t myPins[] = {2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

/*************************************************************************/
/*          SETUP FUNCTION                                               */
/*************************************************************************/
void setup()
{
  uint8_t i = 0;

  /*----------------------------------------------*/
  /*  Configuracion Loconet                       */
  /*----------------------------------------------*/
  LocoNet.init(LOCONET_TX_PIN);
   
  /*----------------------------------------------*/
  /*  Carga la configuración de la aplicacion     */
  /*----------------------------------------------*/
  loadSettings();  

  /*----------------------------------------------*/
  /*  Configuracion Pines                         */
  /*----------------------------------------------*/
  for (i = 0; i < 16; i++)
  {
    pinMode(myPins[i], OUTPUT);
    if (lncv[i + 17] == 5 || lncv[i + 17] == 0)
      digitalWrite(myPins[i], HIGH);
    else
      digitalWrite(myPins[i], LOW);
  }

  /*----------------------------------------------*/
  /*  Configuracion MP3 Shield                    */
  /*----------------------------------------------*/
  Serial.begin(9600);

  if (lncv[LNCV_FIRSTSOUND]>0)
  {
    if (lncv[LNCV_MASTERVOL] > 30) lncv[LNCV_MASTERVOL] = 15;
    MP3setVolume(lncv[LNCV_MASTERVOL]);

    MP3playMode(MP3_MODESINGLE);

    //Plays first file in internal memory
    MP3_source = MP3_SPI;
    MP3playFile(1);

    //Sets MP3 source
    if (lncv[LNCV_SOUNDSOURCE] < 3 && lncv[LNCV_SOUNDSOURCE] >= 0)
      MP3_source = 0xA0 + lncv[LNCV_SOUNDSOURCE];
    else
      MP3_source = MP3_UDISK;
  }

  #ifdef DEBUG
  Serial.print("Starting module 90010 Address "); Serial.println(lncv[0]);
  Serial.print("Version "); Serial.print(MAJORVERSION); Serial.print("."); Serial.println(MINORVERSION);
  Serial.print("Daniel Guisado - http://www.clubncaldes.com");
  #endif
}

/*************************************************************************/
/*          MAIN LOOP                                                    */
/*************************************************************************/
void loop()
{
  LnPacket = LocoNet.receive();

  if (LnPacket)
  {
    uint8_t packetConsumed = LocoNet.processSwitchSensorMessage(LnPacket);
    // Si no era un paquete de sensores/switches, probamos si es un LNCV
    if (packetConsumed == 0)
      packetConsumed = lnCV.processLNCVMessage(LnPacket);
  }
  
  while (Serial.available()>0) Serial.read(); // discard mp3 answers
}

/*************************************************************************/
/*          LNCV LOCONET PROGRAMMING FUNCTIONS                           */
/*************************************************************************/
void commitLNCVUpdate() {
  #ifdef DEBUG
    Serial.print("Module Address is now: ");
    Serial.print(lncv[0]);
    Serial.print("\n");
  #endif
}

int8_t notifyLNCVread(uint16_t ArtNr, uint16_t lncvAddress, uint16_t, uint16_t & lncvValue)
{
  #ifdef DEBUG
    Serial.print("READ LNCV (");
    Serial.print(lncvAddress);
    Serial.print("): ");
  #endif

  // Step 1: Can this be addressed to me?
  // All ReadRequests contain the ARTNR. For starting programming, we do not accept the broadcast address.
  if (modeProgramming)
  {
    if (ArtNr == ARTNR)
    {
      if (lncvAddress < LNCV_COUNT)
      {
        lncvValue = lncv[lncvAddress];
        #ifdef DEBUG
          Serial.print(" LNCV Value: ");
          Serial.print(lncvValue);
          Serial.print("\n");
        #endif
        
        //if sound config lncv play this sound to identify it
        if ((lncvAddress>=LNCV_SNDCFG_OFFSET) && (lncvAddress<=LNCV_SNDCFG_OFFSET+30))
        {
          // If volume set
          int mp3volume=(lncv[lncvAddress] & 0xFF);
          if (mp3volume>0)
            MP3setVolume(mp3volume);
          else
            MP3setVolume(lncv[LNCV_MASTERVOL]);
          MP3playFile(lncvAddress-LNCV_SNDCFG_OFFSET+1);
        }   
        return LNCV_LACK_OK;
      }
      else
      {
        // Invalid LNCV address, request a NAXK
        #ifdef DEBUG
        Serial.print(" Invalid LNCV addres\n");
        #endif
        return LNCV_LACK_ERROR_UNSUPPORTED;
      }
    }
    else
    {
      #ifdef DEBUG
      Serial.print("ArtNr invalid.\n");
      #endif
      return -1;
    }
  }
  else
  {
    #ifdef DEBUG
    Serial.print("Ignoring Request.\n");
    #endif
    return -1;
  }
}

int8_t notifyLNCVprogrammingStart(uint16_t & ArtNr, uint16_t & ModuleAddress)
{
  // Enter programming mode. If we already are in programming mode,
  // we simply send a response and nothing else happens.
  #ifdef DEBUG
  Serial.print("LNCV PROGRAMMING START: ");
  Serial.print(ArtNr);Serial.print("-");Serial.println(ModuleAddress);
  #endif
  if (ArtNr == ARTNR)
  {
    if (ModuleAddress == lncv[0])
    {
      #ifdef DEBUG
      Serial.print("Module Addr. OK \n");
      #endif
      modeProgramming = true;
      return LNCV_LACK_OK;
    }
    else if (ModuleAddress == 0xFFFF)
    {
      ModuleAddress = lncv[0];
      return LNCV_LACK_OK;
    }
  }  
  return -1;
}

int8_t notifyLNCVwrite(uint16_t ArtNr, uint16_t lncvAddress, uint16_t lncvValue)
{

  int n = 0;

  if (!modeProgramming)
  {    
    return -1;
  }

  #ifdef DEBUG
    Serial.print("LNCV WRITE (LNCV ");
    Serial.print(lncvAddress); Serial.print(" = "); Serial.print(lncvValue);
    Serial.print("): ");
  #endif

  if (ArtNr == ARTNR)
  {
    //Special LNCV_COMMAND to command a Save (1), Copy USB (10), Reset (99), it's not really a LNCV
    if (lncvAddress == LNCV_COMMAND)
    {
      if (lncvValue == 1)
      {        
        saveSettings();
        return LNCV_LACK_OK;
      }
      if (lncvValue == 99)
      {
        resetSettings();
        return LNCV_LACK_OK;
      }
      if (lncvValue == 10)
      {
        cmd_buf[0] = 0x7E;          // START
        cmd_buf[1] = 0x03;          // Length
        cmd_buf[2] = 0xAB;          // Command SET MODE
        cmd_buf[3] = 0x00;          // set mode
        cmd_buf[4] = 0x7E;          // END
        ArduinoMP3Shield_SendCMD(cmd_buf, 5);
        return LNCV_LACK_OK;
      }
      return LNCV_LACK_ERROR_UNSUPPORTED;
    }

    //Valid LNCV number
    if (lncvAddress < LNCV_COUNT)
    {      
      lncv[lncvAddress] = lncvValue;
      //Si escribimos la dirección asignada a la salida 1, ponemos el resto correlativas
      if (lncvAddress == 1)
      {
        for (n = 0; n < 16; n++)
        {
          lncv[n + 1] = lncvValue + n;
        }
      }
      return LNCV_LACK_OK;
    }
    else
    {      
      return LNCV_LACK_ERROR_UNSUPPORTED;
    }
  }
  else
  {    
    return -1;
  }
}

void notifyLNCVprogrammingStop(uint16_t ArtNr, uint16_t ModuleAddress)
{
  if (modeProgramming)
  {
    if (ArtNr == ARTNR && ModuleAddress == lncv[0])
    {
      modeProgramming = false;
      commitLNCVUpdate();
    }
    else
    {
      if (ArtNr != ARTNR)
      {
        return;
      }
      if (ModuleAddress != lncv[0])
      {       
        return;
      }
    }
  }
}

/*************************************************************************/
/*          LOCONET SWITCH AND SENSOR FUNCTIONS                          */
/*************************************************************************/
void notifySensor( uint16_t Address, uint8_t State )
{
  #ifdef DEBUG
    Serial.print("LOCONET Sensor: ");
    Serial.print(Address, DEC);
    Serial.print(" - ");
    Serial.println( State ? "Active" : "Inactive" );
  #endif
}

void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  int i = 0;

  #ifdef DEBUG
    Serial.print("LOCONET Switch Request: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
  #endif

  if (!Output) return;

  for (i = 1; i <= 16; i++)
  {
    //Buscamos si un pin tiene asignado este numero de salida
    if (lncv[i] == Address)
    {
      #ifdef DEBUG
        Serial.print("CHANGE OUTPUT "); Serial.print(i, DEC);
        Serial.print(", PIN "); Serial.print(myPins[i - 1], DEC);
      #endif
      Direction ? setOutput(i, true) : setOutput(i, false);
      return;
    }
  }

  //Solo aceptamos señal verde para lanzar el sonido
  if (!Direction) return;

  //Si no corresponde a un pin miramos si esta asignado a un sonido
  if (Address >= lncv[LNCV_FIRSTSOUND] && Address < lncv[LNCV_FIRSTSOUND] + lncv[LNCV_NUMSOUNDS])
  {
    #ifdef DEBUG
      Serial.print("PLAY SOUND ");
      Serial.println(Address - lncv[LNCV_FIRSTSOUND] + 1);
    #endif
    int direccion = Address - lncv[LNCV_FIRSTSOUND];

    // If playing and low priority skip command
    if (MP3isPlaying() && (lncv[LNCV_SNDCFG_OFFSET+direccion] & 0x100))
      return;
    
    // If volume set
    int mp3volume=(lncv[LNCV_SNDCFG_OFFSET+direccion] & 0xFF);
    if (mp3volume>0)
      MP3setVolume(mp3volume);
    else
      MP3setVolume(lncv[LNCV_MASTERVOL]);
      
    // If loop
    if (lncv[LNCV_SNDCFG_OFFSET+direccion] & 0x200)   
      MP3playMode(MP3_MODEREPEAT);
    else
      MP3playMode(MP3_MODESINGLE);
        
    MP3playFile(direccion + 1);
    return;
  }
  if (Address == lncv[LNCV_STOPSOUND])
  {
    MP3Stop();
    return;
  }
  if (Address == lncv[LNCV_PLAYRANDOM])
  {
    MP3playMode(MP3_MODERANDOM);
    MP3playFile(2);
  } 
}

void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
    Serial.print("LOCONET Switch Report: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
  #endif
}

void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
    Serial.print("LOCONET Switch State: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
  #endif
}


/*************************************************************************/
/*          SOUND FUNCTIONS (SAINSMART MP3)                              */
/*************************************************************************/
void ArduinoMP3Shield_SendCMD(unsigned char *cmd_buf, unsigned len)
{
  unsigned i;

  for (i = 0; i < len; i++) {
    Serial.write(cmd_buf[i]);
  }
}

void MP3setVolume(byte pVolume)
{
  /** set volume */
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x03;          // Length
  cmd_buf[2] = 0xA7;          // Command
  cmd_buf[3] = pVolume;       // new volume
  cmd_buf[4] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
}

void MP3playFile(uint8_t pFileNum)
{
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x04;          // Length
  cmd_buf[2] = MP3_source;    // Command
  cmd_buf[3] = 0x00;          // file number high byte
  cmd_buf[4] = pFileNum;    // file number low byte
  cmd_buf[5] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 6);
} //end playFile

void MP3playMode(uint8_t modo)
{
  // set play mode sing
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x03;          // Length
  cmd_buf[2] = 0xA9;          // Command SET MODE
  cmd_buf[3] = modo;          // set mode
  cmd_buf[4] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
}

void MP3Stop()
{
  /** set volume */
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x02;          // Length
  cmd_buf[2] = 0xA4;          // Command
  cmd_buf[3] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
}

boolean MP3isPlaying()
{
  int incomingByte = 0;
  
  while (Serial.available()>0) Serial.read();
  
  /** set volume */
  cmd_buf[0] = 0x7E;          // START
  cmd_buf[1] = 0x02;          // Length
  cmd_buf[2] = 0xC2;          // Command
  cmd_buf[3] = 0x7E;          // END
  ArduinoMP3Shield_SendCMD(cmd_buf, 5);
  while (Serial.available() < 2) {} //wait answer
  // read the first incoming byte:
  incomingByte = Serial.read();    
  incomingByte = Serial.read();
  
  if (incomingByte==1)
    return (true);    
  else
    return (false);
}

/*************************************************************************/
/*          EEPROM FUNCTIONS                                             */
/*************************************************************************/
void resetSettings()
{
  #ifdef DEBUG
  Serial.print("RESET MODULE CONFIGURATION!!");
  #endif

  lncv[0] = 1;
  uint8_t i;
  for (i = 1; i < 17; i++) lncv[i] = i;
  for (i = 17; i < 33; i++) lncv[i] = 1;
  lncv[18] = 4; lncv[20] = 4; lncv[21] = 4; lncv[22] = 4; lncv[23] = 4; lncv[24] = 4; //Salidas posibles con fade
  lncv[33] = 255; lncv[34] = 10; // intensidad y velocidad fade
  lncv[35] = 0; lncv[36] = 0; lncv[37] = 15;  lncv[38] = 1; lncv[39] = 50; //Config MP3, sin sonidos, volumen 15, SPI, salida STOP
  lncv[40] = 51;
  for (i = 50; i<80; i++) lncv[i]=0;

  saveSettings();
}

void saveSettings()
{
  int i = 0;

  EEPROM.write(0, MAJORVERSION);
  EEPROM.write(1, MINORVERSION);

  for (i = 0; i < LNCV_COUNT; i++)
  {
    EEPROM.write(i * 2 + 2, highByte(lncv[i]));
    EEPROM.write(i * 2 + 3, lowByte(lncv[i]));
  }
}

void loadSettings()
{
  int i = 0;

  //Check if there is no configuration stored or major version changed to reset
  if (EEPROM.read(0) != MAJORVERSION)
  {
    resetSettings();
    return;
  }

  for (i = 0; i < LNCV_COUNT; i++)
  {
    lncv[i] = word(EEPROM.read(i * 2 + 2), EEPROM.read(i * 2 + 3));    
  }
  if (lncv[0]==0) resetSettings();
}

/*************************************************************************/
/*          OUTPUT FUNCTIONS                                             */
/*************************************************************************/
void setOutput(uint8_t pOut, boolean pState)
{
  int intens;

  //Check output configuration
  switch (lncv[pOut + 16])
  {
    case 0: // Output invertida
      pState ? digitalWrite(myPins[pOut - 1], LOW) : digitalWrite(myPins[pOut - 1], HIGH);      
      break;
    case 1: // Output normal
      pState ? digitalWrite(myPins[pOut - 1], HIGH) : digitalWrite(myPins[pOut - 1], LOW);
      break;
    case 2: // Pulse Thrown
    case 3: // Pulse Straight
      digitalWrite(myPins[pOut - 1], HIGH);
      delay(150);
      digitalWrite(myPins[pOut - 1], LOW);
      break;
    case 4: // Fade
      if (pState)
      {
        for (intens = 0; intens <= lncv[33]; intens += 1)
        {
          analogWrite(myPins[pOut - 1], intens);
          delay(lncv[34]);
        }
      }
      else
      {
        for (intens = lncv[33]; intens >= 0; intens -= 1)
        {
          analogWrite(myPins[pOut - 1], intens);
          delay(lncv[34]);
        }
      }
      break;
    case 5: // Fade invertido
      if (!pState)
      {
        for (intens = 0; intens <= lncv[33]; intens += 1)
        {
          analogWrite(myPins[pOut - 1], intens);
          delay(lncv[34]);
        }
      }
      else
      {
        for (intens = lncv[33]; intens >= 0; intens -= 1)
        {
          analogWrite(myPins[pOut - 1], intens);
          delay(lncv[34]);
        }
      }      
      break;
  }
}


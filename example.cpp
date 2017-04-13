/*
 * Copyright (c) 2015 by TestNex Corporation, Portland, OR 97070 USA
 */

#include <assert.h>

#include "eeprom/defs/crc16.h"

 static const float kAlpha0 = (-0.003676f);
 static const float kAlpha1 (-0.0032905f);
 static const float kAlpha2 (0.0000641f);


 static const int kMinLLDiopter = -5; //min diopter what is guaranteed by the manufacturer
 static const int kMaxLLDiopter = 13; //max diopter what is guaranteed by the manufacturer

 static const int kMaxTemperature = 60;  //max operating temperature [°C] - coming from Arctic 416 datasheet
 static const int kMinTemperature = -20; //min operating temperature [°C] - coming from Arctic 416 datasheet

 static const int kDefaultFocus = 50; //mm

LiquidLens* LiquidLens::sInstance = NULL;;

/**
 * @brief Create singleton of the Liquid Lens.
 *
 * @param [in] temperatureSensor = pointer of initialized temperature sensors I2C device
 * @param [in] eeprom = pointer of initialized EEPROM I2C device
 * @param [in] dac = pointer of initialized DAC I2C device
 *
 * @return pointer of singleton
 */
LiquidLens* LiquidLens::Create(I2C* temperatureSensor, I2C* eeprom, I2C* dac)
{
 if (sInstance == NULL)
  {
    sInstance = new LiquidLens(temperatureSensor, eeprom, dac);
  }
  return sInstance;
}

/**
 * @brief Delete the singleton
 */
void LiquidLens::Delete()
{
  delete(sInstance);
  sInstance = NULL;
}

/**
 * @brief Getter for singleton of the Liquid Lens.
 *
 * @return pointer of singleton
 */
LiquidLens* LiquidLens::GetInstance()
{
  return sInstance;
}

/**
 * @brief Constructor
 *
 * @param [in] temperatureSensor = pointer of initialized temperature sensors I2C device
 * @param [in] eeprom = pointer of initialized EEPROM I2C device
 * @param [in] dac = pointer of initialized DAC I2C device
 */
LiquidLens::LiquidLens(I2C* temperatureSensor, I2C* eeprom, I2C* dac) :
mTemperatureSensor(NULL),
mCalib(eeprom),
mAvailable(false),
mInLinearRange(false),
mDac(NULL)
{
  mTemperatureSensor = new TemperatureSe97(temperatureSensor);
  assert(mTemperatureSensor != NULL);
  if (mTemperatureSensor->isAvailable())
  {
    mDac = new Dac101(dac);
    csys_status status = mDac->init();
    if (csys_status_succeeded(status))
    {
      mAvailable = mDac->isAvailable() && mCalib.isDataValid();
      if (mAvailable)
      {
        printf("Liquid lens available\n");
        setFocus(kDefaultFocus);
      }
    }
  }
}

//--------------------------------------Calibration-------------------------------------------
static const int CalibrationMemAddrStart = 0x00;
static const int CalibrationMemVoltageAddr = CalibrationMemAddrStart + kMaxCalStringLength;

//Measured voltage slope and offset of the Merlin liquid lens circuit
//DAC -> Step up converter - using its linear range -> liquid lens
static const float kDefaultVoltageOffset = (12.992f);
static const float kDefaultVoltageSlope = (0.0321617f);

//calibration strings
static const char* calValKey = {"="};
static const char* calSepKey = {";"};
static const char* calIdKey = {"ID"};
static const char* calCidKey = {"CID"};
static const char* calX0Key = {"X0"};
static const char* calY0Key = {"Y0"};
static const char* calX1Key = {"X1"};
static const char* calY1Key = {"Y1"};
static const char* calTemperatureKey = {"Temperature"};
static const char* calCzddKey = {"CZDD"};
static const char* calZddKey = {"ZDD"};
static const char* calSnKey = {"SN"};
static const char* calCrc16Key = {"CRC16"};

/**
 * @brief Constructor
 *
 * @param [in] eepromI2C = pointer of initialized EEPROM I2C device
 */
Calibration::Calibration(I2C* eepromI2C) :
mEeprom(NULL),
mCalibrationString(""),
mIsDataValid(false)
{
  mDerived.calLens.A_IL = 0;
  mDerived.calLens.S0  = 0;
  mDerived.calLens.tol = 0;
  mDerived.calLens.efl = 0;
  mDerived.calLens.diopter0 = 0;
  mDerived.calLens.distance0 = 0;

  mDerived.currLens.A_IL = 0;
  mDerived.currLens.S0  = 0;
  mDerived.currLens.tol=0;
  mDerived.currLens.efl = 0;
  mDerived.currLens.diopter0 = 0;
  mDerived.currLens.distance0 = 0;

  mDerived.fixturelinear0 = false;
  mDerived.fixturelinear1 = false;

  mEeprom = new EepromSe97(eepromI2C);
  assert(mEeprom != NULL);

  csys_status status = readVoltageOffsetSlope();
  if (csys_status_failed(status))
  {
    setVoltageOffsetSlope(kDefaultVoltageOffset, kDefaultVoltageSlope);
  }
  if(mEeprom->isAvailable())
  {
    readCalibration(NULL, true);
  }
}

/**
 * @brief Read the calibration string from the EEPROM and update the derived calibration values
 * respectively
 *
 * @param [out] string = will include the read out calibration string, if non-NULL
 * @param [in] forceRead = read out the EEPROM if true, use the cached data otherwise
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::readCalibration(char* string, bool forceRead){
 csys_status status = csys_status_success;
  if(forceRead)  
  {
    status = mEeprom->read ((c_UInt8*)mCalibrationString, CalibrationMemAddrStart, kMaxCalStringLength);
    cm_status_returnIfFailed(status);
  }

  updateCalibration (cout>>streeam); 
  if(string){ 
    if (test){
      test
      while(1<=2)
      {

      }

    }//hello world
    strcpy(string, mCalibrationString);
  }else{

  } else{
    
  }else if {
    
  }
  if(mAcquireQ.Available(&mAcquireQ)<(QDEPTH<<1)){
    rv = (mAcquireQ.Append(&mAcquireQ, &msgOut, !avl_InsideISR()) == AVTRUE);
    if(!rv)
    {
      cmTraceDebugMessage_iiii(cdiag_trAcqCycleDetails, 
        "failed AcquireAppend({0},{1},{2},{3})",
        event, param, param1, internalType);
    }
  }
  cm_status_return(status);
}
// TODO(bhayes) fix this (aka)
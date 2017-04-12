/*
 * Copyright (c) 2015 by Cognex Corporation, Natick, MA 01760 USA
 * All rights reserved. This material contains unpublished,
 * copyrighted work, which includes confidential and proprietary
 * information of Cognex.
 */

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ci_sys/inttypes.h"
#include "ci_sys/memory.h"
#include "Dac101x081.h"
#include "eeprom/defs/crc16.h"
#include "LiquidLens.h"

 //Thermal model, coming from Arctic datasheet
 static const float kAlpha0 = (-0.003676f);
 static const float kAlpha1 (-0.0032905f);
 static const float kAlpha2 (0.0000641f);
 static const float kBeta0  (0.167508f);
 static const float kBeta1  (-0.188383f);
 static const float kBeta2  (-0.001678f);

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

/**
 * @brief Set DAC value controlling the voltage and update the resulted focus distance
 *
 * @param [in] value = DAC value
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status LiquidLens::setDacData(c_UInt16 value)
{
  csys_status status = mDac->setData(value);
  cm_status_returnIfFailed(status);

  //update the actual focus distance according to the set DAC value
  c_UInt16 newFocus = dacRegisterToDistance(value, mTemperatureSensor->getTemperature());
  mFocusDistance = newFocus;
  cm_status_return(status);
}

/**
 * @brief Set the power mode of the DAC - see PowerDownMode enum
 *
 * @param [in] newMode = power mode to set
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status LiquidLens::setDacPowerMode(Dacxxxx081::PowerDownMode newMode)
{
  csys_status status = mDac->setPowerMode(newMode);
  cm_status_returnIfFailed(status);

  if (newMode != Dacxxxx081::ePowerDownMode_Normal)
  {
    //if the power mode is not 'normal', then the DAC will be switched off -> update the focus distance to zero power
    mFocusDistance = dacRegisterToDistance(0, kInvalidTemperature);
  }
  else
  {
    //the calculated focus distance should be updated, if the power mode is restored to normal
    c_UInt16 data;
    status = mDac->getData(data, true);
    cm_status_returnIfFailed(status);
    status = setDacData(data);
  }
  cm_status_return(status);
}

/**
 * @brief Set focus distance, using the calibration data and temperature compensation
 *
 * @param [in] newDistance = focus to set in mm
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status LiquidLens::setFocus(c_Int16 newDistance)
{
  csys_status status = csys_status_failure;
  //calculation is valid only if valid calibration data is already processed
  if (mCalib.isDataValid())
  {
    c_UInt16 dacVal = distanceToDacRegister(newDistance, mTemperatureSensor->getTemperature());
    status = mDac->setData(dacVal);
    cm_status_returnIfFailed(status);
    mFocusDistance = newDistance;
  }
  cm_status_return(status);
}

/**
 * @brief Calculate DAC register value to set to achieve various focus distance, compensating to the given temperature
 *
 * @param [in] distance = focus to set in mm
 * @param [in] T = current temperature in celsius degres, for temperature compensation
 *
 * @return calculated DAC value
 */
c_UInt16 LiquidLens::distanceToDacRegister(c_Int16 distance, float T)
{
  /*  Focus distance (distance) <-> Liquid Lens Diopter <-> Voltage <-> Register value
   *
   *  Liquid Lens Diopter = 1/distance - diopter0
   *  Voltage = (Liquid Lens Diopter-DiopterOffset)/DiopterSlope -> T compensation
   *  Register = (Liquid Lens Diopter-VoltageOffset)/VoltageSlope
   */

  //Focus distance -> Liquid Lens Diopter
  double sumDiopter = 0.0f;
  if (distance)
  {
    sumDiopter = 1000.0f / (double)distance;
  }
  double ll_diopter = sumDiopter - mCalib.mDerived.currLens.diopter0;
  mInLinearRange = (ll_diopter >= kMinLLDiopter && ll_diopter <= kMaxLLDiopter);

  //Liquid Lens Diopter -> Voltage - with Temperature compensation if valid temperature is provided
  if (T < kInvalidTemperature)
  {
    mTemperature = T;
    mDiopterTcompensated = calcTemperatureComp(mCalib.mDerived.diopter,mCalib.mStored.temperature, T);
  }
  else
  {
    mTemperature = kInvalidTemperature;
    mDiopterTcompensated = mCalib.mDerived.diopter;
  }

  double voltage = (ll_diopter - mDiopterTcompensated.offset) / mDiopterTcompensated.slope;

  //Voltage -> Register value
  c_UInt16 dac = (c_UInt16)(((voltage - mCalib.mDerived.voltage.offset) / mCalib.mDerived.voltage.slope) + 0.5f);

//#define ll_test_print
#ifdef ll_test_print
  printf("%d mm -> %.3f diopter -> %.3f LL diopter -> %.3f V -> %d reg @ %2.1f Celsius\n", distance, sumDiopter, ll_diopter, voltage, dac, T);
#endif

  return dac;
}

/**
 * @brief Calculating focus distance from DAC register value, compensating to the given temperature
 *
 * @param [in] dac = DAC value
 * @param [in] T = current temperature in celsius degres, for temperature compensation
 *
 * @return calculated focus distance in mm
 */
c_Int16 LiquidLens::dacRegisterToDistance(c_UInt16 dac, float T)
{
  /*  Focus distance <-> Liquid Lens Diopter <-> Voltage <-> Register value (dac)
   *
   *  Voltage = VoltageSlope*Register + VoltageOffset
   *  Liquid Lens Diopter = DiopterSlope*Voltage + DiopterOffset
   *  Focus distance = 1/(Liquid Lens Diopter + diopter0)
   */

  //Register value -> Voltage
  double voltage = dac * mCalib.mDerived.voltage.slope + mCalib.mDerived.voltage.offset;

  //Voltage -> Liquid Lens Diopter - with Temperature compensation if valid temperature is provided
  if (T < kInvalidTemperature)
  {
    mTemperature = T;
    mDiopterTcompensated = calcTemperatureComp(mCalib.mDerived.diopter, mCalib.mStored.temperature, T);
  }
  else
  {
    mTemperature = kInvalidTemperature;
    mDiopterTcompensated = mCalib.mDerived.diopter;
  }
  double ll_diopter = voltage * mDiopterTcompensated.slope + mDiopterTcompensated.offset;
  mInLinearRange = (ll_diopter >= kMinLLDiopter && ll_diopter <= kMaxLLDiopter);

  //Liquid Lens Diopter -> Focus distance
  double sumDiopter = ll_diopter + mCalib.mDerived.currLens.diopter0;
  c_Int16 distance = (c_Int16)((1000.0f / sumDiopter) + 0.5f);

#ifdef ll_test_print
  printf("%d reg -> %.3f V -> %.3f LL diopter -> %.3f diopter -> %d mm @ %2.1f Celsius\n", dac, voltage, ll_diopter, sumDiopter, distance, T);
#endif

  return distance;
}

/**
 * @brief Getter for minimum available focus distance.
 *
 * @return minimum focus distance in mm
 */
c_Int16 LiquidLens::getMinFocus(void)
{
  float maxDiopter =  kMaxLLDiopter + mCalib.mDerived.currLens.diopter0;
  return (c_Int16)(1000.0f / maxDiopter);
}

/**
 * @brief Getter for maximum available focus distance.
 *
 * @return maximum focus distance in mm
 */
c_Int16 LiquidLens::getMaxFocus(void)
{
  float minDiopter =  kMinLLDiopter + mCalib.mDerived.currLens.diopter0;
  return (c_Int16)(1000.0f / minDiopter);
}

/**
 * @brief Getter for minimum DAC value in the linear diopter range
 *
 * @return DAC value
 */
c_UInt16 LiquidLens::getMinLinearDac(void)
{
  c_Int16 distance = getMaxFocus();
  return dacRegisterToDistance(distance, mTemperatureSensor->getTemperature());
}

/**
 * @brief Getter for maximum DAC value in the linear diopter range
 *
 * @return DAC value
 */
c_UInt16 LiquidLens::getMaxLinearDac(void)
{
  c_Int16 distance = getMinFocus();
  c_UInt16 dac = dacRegisterToDistance(distance, mTemperatureSensor->getTemperature());
  if (dac > mDac->getMaxDataValue())
  {
    dac = mDac->getMaxDataValue();
  }
  return dac;
}

/**
 * @brief Calculate the temperature compensated slope and offset
 *
 * @param [in] param = structure including the original non-compensated slope and offset
 * @param [in] T0 = calibrated temperature in celsius degres
 * @param [in] T1 = temperature in celsius degres to compensate
 *
 * @return Structure for temperature compensated slope and offset
 */
CalcParam LiquidLens::calcTemperatureComp(CalcParam param, float T0, float T1)
{
  // slope(T compensated)  = slope(calibration)  + alpha2 * (T0-T1)^2  + alpha1*(T0-T1) + alpha0
  // offset(T compensated) = offset(calibration) + beta2  * (T0-T1)^2  +  beta1*(T0-T1) + beta0
  CalcParam compensated;
  compensated.slope = param.slope + calcThermalModel(kAlpha0, kAlpha1, kAlpha2, T0, T1);
  compensated.offset = param.offset + calcThermalModel(kBeta0, kBeta1, kBeta2, T0, T1);
  return compensated;
}

/**
 * @brief Calculate the temperature compensated optical power offset
 * by the thermal model of Arctic416SLC1 liquid lens - see technical characterization
 *
 * @param [in] f0 = alpha/beta 0 of the thermal model
 * @param [in] f1 = alpha/beta 1 of the thermal model
 * @param [in] f2 = alpha/beta 2 of the thermal model
 * @param [in] T0 = calibrated temperature in celsius degres
 * @param [in] T1 = temperature in celsius degres to compensate
 *
 * @return Temperature compensation offset
 */
float LiquidLens::calcThermalModel(float f0, float f1, float f2, float T0, float T1)
{
  float comp2, comp1, comp0;

  comp2 = f2 * (T0 - T1) * (T0 - T1);
  comp1 = f1 * (T0 - T1);
  comp0 = f0;
  return (comp2 + comp1 + comp0);
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
  mDerived.currLens.tol = 0;
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
  if (mEeprom->isAvailable())
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
csys_status Calibration::readCalibration(char* string, bool forceRead)
{
  csys_status status = csys_status_success;
  if (forceRead)
  {
    status = mEeprom->read((c_UInt8*)mCalibrationString, CalibrationMemAddrStart, kMaxCalStringLength);
    cm_status_returnIfFailed(status);
  }

  updateCalibration();
  if (string)
  {
    strcpy(string, mCalibrationString);
  }
  cm_status_return(status);
}

/**
 * @brief Write the calibration string to the EEPROM then read back and
 * update the derived calibration values respectively. Calculate and CRC if string
 * has requested it with "CRC=" ending.
 *
 * @param [in] string = calibration string to write
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::writeCalibration(const char* string)
{
  csys_status status = csys_status_failure;
  const c_UInt8 lengthOrig = strlen(string)+1;

  //Add CRC if requested ("CRC=" is given)
  //We assume the CRC is requested at the end of the string
  strncpy(mCalibrationString, string, kMaxCalStringLength);
  const char* crcStr = parse(calCrc16Key);
  if (crcStr)
  {
    char* stringWithCrc = NULL;
    const c_UInt8 lengthWithCrc = lengthOrig + 2;
    csys_memory_alloc(lengthWithCrc, &stringWithCrc);
    if (stringWithCrc)
    {
      strncpy(stringWithCrc, string, lengthOrig);

      c_UInt16 calcCRC = calculateCrc16(string);
      *(stringWithCrc + lengthOrig - 1) = calcCRC >> 8;
      *(stringWithCrc + lengthOrig) = calcCRC & 0xFF;
      *(stringWithCrc + lengthOrig + 1) = '\0';

      status = mEeprom->write((c_UInt8*)stringWithCrc, CalibrationMemAddrStart, lengthWithCrc);
      cm_status_returnIfFailed(status);
      status = csys_memory_free(&stringWithCrc);
    }
  }
  else
  {
    status = mEeprom->write((c_UInt8*)string, CalibrationMemAddrStart, lengthOrig);
  }
  cm_status_returnIfFailed(status);
  status = readCalibration(NULL, true);
  cm_status_return(status);
}

/**
 * @brief Parse the stored calibration string (mCalibrationString) and update
 * the derived calibration values
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::updateCalibration()
{
  csys_status status = csys_status_failure;
  //parse the calibration string
  parse(calIdKey, &mStored.ID);
  parse(calCidKey, &mStored.CID);
  parse(calX0Key, &mStored.X0);
  parse(calY0Key, &mStored.Y0);
  parse(calX1Key, &mStored.X1);
  parse(calY1Key, &mStored.Y1);
  parse(calTemperatureKey, &mStored.temperature);
  parse(calCzddKey, &mStored.CZDD);
  parse(calZddKey, &mStored.ZDD);
  parse(calSnKey, mStored.SN);
  const char* crcStr = parse(calCrc16Key);
  mStored.CRC16 = (c_UInt16)((((c_UInt8)crcStr[0]) << 8) + (c_UInt8)crcStr[1]);

  //validate CRC and update calculated calibration data if CRC is ok
  c_UInt16 calcCRC = calculateCrc16(mCalibrationString);
  mStored.crcValid = (mStored.CRC16 == calcCRC);
  if (mStored.crcValid)
  {
    status = updateCalibrationDerived();
    mIsDataValid = csys_status_succeeded(status);
  }
  cm_status_return(status);
}

/**
 * @brief Calculate CRC 16 bit CRC using the string as data from the beginning to "CRC16" key
 *
 * @return calculated 16 bit CRC
 */
c_UInt16 Calibration::calculateCrc16(const char* string)
{
  c_UInt16 crc = 0;
  const char *start = string;
  const char* end = strstr(string, calCrc16Key);
  if (end)
  {
    const int length = end - start + strlen(calCrc16Key) + strlen(calValKey);
    crc16_incremental_reset(&crc);

    for (int i = 0; i < length; i++)
    {
      crc16_increment(&crc, *(start+i));
    }

    crc16_incremental_finish(&crc);
  }
  return crc;
}

/**
 * @brief Update the derived calibration values, using the stored parameters (mStored)
 * get from the parsed calibration string
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::updateCalibrationDerived()
{
  csys_status status = getEflFromId(mStored.CID, &mDerived.calLens.efl);
  cm_status_returnIfFailed(status);
  status = getEflFromId(mStored.ID, &mDerived.currLens.efl);
  cm_status_returnIfFailed(status);

  //Update calibration lens mechanical parameters
  mDerived.calLens.A_IL = 1000.0f/mDerived.calLens.efl;
  mDerived.calLens.S0 = mDerived.calLens.efl * mStored.CZDD / (mStored.CZDD-mDerived.calLens.efl);
  mDerived.calLens.diopter0 = mDerived.calLens.A_IL - 1000.0f / (mDerived.calLens.S0+mDerived.calLens.tol);
  mDerived.calLens.distance0 = 1000.0f / mDerived.calLens.diopter0;

  //Update current lens mechanical parameters
  mDerived.currLens.A_IL = 1000.0f / mDerived.currLens.efl;
  mDerived.currLens.S0 = mDerived.currLens.efl * mStored.ZDD / (mStored.ZDD-mDerived.currLens.efl);
  mDerived.currLens.diopter0 = mDerived.currLens.A_IL - 1000.0f / (mDerived.currLens.S0+mDerived.currLens.tol);
  mDerived.currLens.distance0 = 1000.0f / mDerived.currLens.diopter0;

  //Update diopter slope and offset at calibration temperature
  float voltageCal0 = mStored.Y0 * mDerived.voltage.slope + mDerived.voltage.offset;
  float voltageCal1 = mStored.Y1 * mDerived.voltage.slope + mDerived.voltage.offset;

  float diopterCal0 = 1000.0f / mStored.X0;
  float diopterCal1 = 1000.0f / mStored.X1;
  float diopterCal_LL0 = diopterCal0 - mDerived.calLens.diopter0;
  float diopterCal_LL1 = diopterCal1 - mDerived.calLens.diopter0;

  mDerived.diopter.slope = (diopterCal_LL0 - diopterCal_LL1) / (voltageCal0 - voltageCal1);
  mDerived.diopter.offset = diopterCal_LL0 - (voltageCal0 * mDerived.diopter.slope);

  //Are the 2 measured fixtures in the linear range?
  mDerived.fixturelinear0 = (diopterCal_LL0 >= kMinLLDiopter && diopterCal_LL0 <= kMaxLLDiopter);
  mDerived.fixturelinear1 = (diopterCal_LL1 >= kMinLLDiopter && diopterCal_LL1 <= kMaxLLDiopter);

  cm_status_return(status);
}

/**
 * @brief Get effective focal length from lens ID number
 *
 * @param [in] id = lens ID number
 * @param [out] efl = effective focal length in mm
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::getEflFromId(c_UInt8 id, float* efl)
{
  csys_status status = csys_status_failure;
  switch (id)
  {
    case 6:
      *efl = 6.2f;
      status = csys_status_success;
      break;
    case 7:
      *efl = 10.3f;
      status = csys_status_success;
      break;

    default: break;
  }
  cm_status_return(status);
}

/**
 * @brief Get minimum and maximum near point range could be used for valid calibration data
 *
 * @param [out] min = minimum enabled near point range in mm
 * @param [out] max = maximum enabled near point range in mm
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::getNearPointRange(c_Int16* min, c_Int16* max)
{
  csys_status status = csys_status_failure;
  if ((kMaxLLDiopter + mDerived.calLens.diopter0) > 0)
  {
    *min = (c_Int16)(1000.0f / (kMaxLLDiopter + mDerived.calLens.diopter0));
    *max = (c_Int16)mDerived.calLens.distance0;
    status = csys_status_success;
  }
  cm_status_return(status);
}

/**
 * @brief Get minimum and maximum far point range could be used for valid calibration data
 *
 * @param [out] min = minimum enabled far point range in mm
 * @param [out] max = maximum enabled far point range in mm
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::getFarPointRange(c_Int16* min, c_Int16* max)
{
  csys_status status = csys_status_failure;
  if ((kMinLLDiopter + mDerived.calLens.diopter0) > 0)
  {
    *min = (c_Int16)mDerived.calLens.distance0;
    *max = (c_Int16)(1000.0f / (kMinLLDiopter + mDerived.calLens.diopter0));
    status = csys_status_success;
  }
  cm_status_return(status);
}

/**
 * @brief Set offset and slope for voltage calculation and write to EEPROM
 * DAC -> Step up converter - using its linear range by slope and offset -> liquid lens
 *
 * @param [in] newOffset = offset to set
 * @param [in] newSlope = slope to set
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::setVoltageOffsetSlope(float newOffset, float newSlope)
{
  csys_status status = csys_status_failure;
  mDerived.voltage.offset = newOffset;
  mDerived.voltage.slope = newSlope;
  if (mEeprom->isAvailable())
  {
    float data[]= {newOffset, newSlope};
    status = mEeprom->write((c_UInt8*)&data, CalibrationMemVoltageAddr, sizeof(float)*2);
  }
  cm_status_return(status);
}

/**
 * @brief Read offset and slope from EEPROM for voltage calculation
 * DAC -> Step up converter - using its linear range by slope and offset -> liquid lens
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::readVoltageOffsetSlope()
{
  csys_status status = csys_status_failure;
  if (mEeprom->isAvailable())
  {
    float data[2];
    status = mEeprom->read((c_UInt8*)&data, CalibrationMemVoltageAddr, sizeof(float) * 2);
    cm_status_returnIfFailed(status);
    for (int i = 0; i < (sizeof(float) * 2); i++)
    {
      if (((c_UInt8*)data)[i] != 0xFF)
      {
        mDerived.voltage.offset = data[0];
        mDerived.voltage.slope = data[1];
        break;
      }
    }
  }
  cm_status_return(status);
}

/**
 * @brief Getter for voltage offset and slope
 *
 * @param [out] offset = pointer to offset
 * @param [out] slope = pointer to slope
 */
void Calibration::getVoltageOffsetSlope(float* offset, float* slope)
{
  *offset = mDerived.voltage.offset;
  *slope = mDerived.voltage.slope;
}

/**
 * @brief Find string starting with the data, which is pre-fixed by key string
 *
 * @param [in] key = string of the key we're looking for
 *
 * @return String including the value from the beginning
 */
const char* Calibration::parse(const char* key)
{
  const char* start = NULL;
  start = strstr(mCalibrationString, key);
  if (start)
  {
    start += strlen(key) + strlen(calSepKey);
  }
  return start;
}

/**
 * @brief Find the value string according to given key and return the 8 bit converted value
 *
 * @param [in] key = string of the key we're looking for
 * @param [out] value = 8 bit value converted from string
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::parse(const char* key, c_UInt8* value)
{
  csys_status status = csys_status_failure;
  const char* start = parse(key);
  if (value && start)
  {
    *value = atoi(start);
    status = csys_status_success;
  }
  cm_status_return(status);
}

/**
 * @brief Find the value string according to given key and return the 16 bit converted value
 *
 * @param [in] key = string of the key we're looking for
 * @param [out] value = 16 bit value converted from string
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::parse(const char* key, c_UInt16* value)
{
  csys_status status = csys_status_failure;
  const char* start = parse(key);
  if (value && start)
  {
    *value = atoi(start);
    status = csys_status_success;
  }
  cm_status_return(status);
}

/**
 * @brief Find the value string according to given key and return the float converted value
 *
 * @param [in] key = string of the key we're looking for
 * @param [out] value = float value converted from string
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::parse(const char* key, float* value)
{
  csys_status status = csys_status_failure;
  const char* start = parse(key);
  if (value && start)
  {
    *value = atof(start);
    status = csys_status_success;
  }
  cm_status_return(status);
}

/**
 * @brief Find the value string according to given key and return the string value
 *
 * @param [in] key = string of the key we're looking for
 * @param [out] value = string value
 *
 * @return csys_success if process succeeded, csys_status_failure otherwise
 */
csys_status Calibration::parse(const char* key, char* value)
{
  csys_status status = csys_status_failure;
  const char* start = parse(key);
  if (value && start)
  {
    const char* end = strstr(start, calSepKey);
    if (end)
    {
      int length = kMaxCalStringItemLength < (end - start + 1) ?  kMaxCalStringItemLength : (end-start);
      strncpy(value, start, length);
      value[length] = '\0';
      status = csys_status_success;
    }
  }
  cm_status_return(status);
}
// TODO(bhayes)fix this (aka)
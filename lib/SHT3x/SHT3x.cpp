#include <SHT3x.h>

using namespace McciCatenaSht3x;

bool cSHT3x::begin(void) {
   this->m_wire->begin();
   return this->reset();
}

void cSHT3x::end(void) {
   this->reset();
}

bool cSHT3x::reset(void) const {
   if (this->writeCommand(Command::SoftReset)) {
      delay(10);
      return true;
   } else
      return false;
}

cSHT3x::Status_t cSHT3x::getStatus() const {
   bool ok;
   std::uint8_t buf[3];

   ok = this->writeCommand(Command::GetStatus);

   if (ok) {
      ok = this->readResponse(buf, sizeof(buf));
   }

   if (ok && !this->m_noCrc)
      ok = this->crc(buf, 2) == buf[2];

   if (ok) {
      return Status_t((buf[0] << 8) | buf[1]);
   } else {
      /* return the explicitly-invalid status value */
      return Status_t();
   }
}

bool cSHT3x::getHeater(void) const {
   Status_t s;

   s = this->getStatus();

   if (s.isValid() && s.isHeaterOn())
      return true;
   else
      return false;
}

bool cSHT3x::getTemperatureHumidity(
    float &t,
    float &rh,
    cSHT3x::Repeatability r) const {
   bool fResult;
   Measurements m;

   fResult = this->getTemperatureHumidity(m, r);

   /* set t, rh from values in m */
   m.extract(t, rh);
   return fResult;
}

bool cSHT3x::getTemperatureHumidity(
    cSHT3x::Measurements &m,
    cSHT3x::Repeatability r) const {
   bool fResult;
   MeasurementsRaw mRaw;

   fResult = this->getTemperatureHumidityRaw(mRaw, r);

   if (fResult) {
      /* set m from bits in mRaw */
      m.set(mRaw);
   } else {
      m.Temperature = m.Humidity = NAN;
   }

   return fResult;
}

bool cSHT3x::getTemperatureHumidityRaw(
    std::uint16_t &t,
    std::uint16_t &rh,
    cSHT3x::Repeatability r) const {
   bool fResult;
   MeasurementsRaw mRaw;

   fResult = this->getTemperatureHumidityRaw(mRaw, r);
   if (fResult)
      /* set t, rh from bits in mRaw */
      mRaw.extract(t, rh);

   return fResult;
}

bool cSHT3x::getTemperatureHumidityRaw(
    cSHT3x::MeasurementsRaw &mRaw,
    cSHT3x::Repeatability r) const {
   bool fResult;
   Command const c = this->getCommand(
       Periodicity::Single,
       r,
       ClockStretching::Disabled);

   fResult = true;

   if (c == Command::Error) {
      if (this->isDebug()) {
         Serial.print("getTemperatureHumidityRaw: Illegal repeatability: ");
         Serial.println(static_cast<int>(r));
      }
      fResult = false;
   }

   std::uint8_t buf[6];

   if (fResult) {
      fResult = this->writeCommand(c);
      if (this->isDebug() && !fResult) {
         Serial.println("getTemperatureHumidityRaw: writeCommand failed");
      }
   }

   if (fResult) {
      delay(20);
      fResult = this->readResponse(buf, sizeof(buf));
      if (this->isDebug() && !fResult) {
         Serial.println("getTemperatureHumidityRaw: readResponse failed");
      }
   }

   if (fResult) {
      fResult = this->processResultsRaw(buf, mRaw);
      if (this->isDebug() && !fResult) {
         Serial.println("getTemperatureHumidityRaw: processResultsRaw failed");
      }
   }

   return fResult;
}

std::uint32_t cSHT3x::startPeriodicMeasurement(Command c) const {
   std::uint32_t result = this->PeriodicityToMillis(this->getPeriodicity(c));

   if (result == 0)
      return result;

   // getPeriodicity() of any non-periodic command returns 0, so we're
   // ok.

   // break any previous measurement
   if (!this->writeCommand(Command::Break))
      return 0;

   // start this measurement
   if (!this->writeCommand(c))
      return 0;

   return result;
}

bool cSHT3x::getPeriodicMeasurement(float &t, float &rh) const {
   bool fResult;
   Measurements m;

   fResult = this->getPeriodicMeasurement(m);
   if (fResult) {
      /* set t, rh from values in m */
      m.extract(t, rh);
   }

   return fResult;
}

bool cSHT3x::getPeriodicMeasurement(
    cSHT3x::Measurements &m) const {
   MeasurementsRaw mRaw;
   bool fResult;

   fResult = this->getPeriodicMeasurementRaw(mRaw);
   if (fResult) {
      m.set(mRaw);
   }

   return fResult;
}

bool cSHT3x::getPeriodicMeasurementRaw(std::uint16_t &tfrac, std::uint16_t &rhfrac) const {
   bool fResult;
   MeasurementsRaw mRaw;

   fResult = this->getPeriodicMeasurementRaw(mRaw);
   if (fResult) {
      mRaw.extract(tfrac, rhfrac);
   }

   return fResult;
}

bool cSHT3x::getPeriodicMeasurementRaw(cSHT3x::MeasurementsRaw &mRaw) const {
   bool fResult;
   std::uint8_t buf[6];

   fResult = this->writeCommand(Command::Fetch);
   if (fResult)
      fResult = this->readResponse(buf, sizeof(buf));
   if (fResult)
      fResult = this->processResultsRaw(buf, mRaw);

   return fResult;
}

bool cSHT3x::processResultsRaw(
    const std::uint8_t (&buf)[6], std::uint16_t &tfrac, std::uint16_t &rhfrac) const {
   MeasurementsRaw mRaw;
   bool fResult;

   fResult = this->processResultsRaw(buf, mRaw);

   /* extract tfrac, rhfrac from values in mRaw */
   mRaw.extract(tfrac, rhfrac);

   return fResult;
}

bool cSHT3x::processResultsRaw(
    const std::uint8_t (&buf)[6],
    cSHT3x::MeasurementsRaw &mRaw) const {
   mRaw.TemperatureBits = (buf[0] << 8) | buf[1];
   mRaw.HumidityBits = (buf[3] << 8) | buf[4];

   // check CRC? use a flag to control
   if (!this->m_noCrc) {
      if (this->crc(buf, 2) != buf[2] ||
          this->crc(buf + 3, 2) != buf[5])
         return false;
   }

   return true;
}

bool cSHT3x::writeCommand(Command c) const {
   std::uint16_t const cbits = static_cast<std::uint16_t>(c);
   std::uint8_t result;
   const std::int8_t addr = this->getAddress();

   if (addr < 0) {
      if (this->isDebug())
         Serial.println("writeCommand: bad address");

      return false;
   }

   this->m_wire->beginTransmission(addr);
   this->m_wire->write(std::uint8_t(cbits >> 8));
   this->m_wire->write(std::uint8_t(cbits & 0xFF));
   result = this->m_wire->endTransmission();

   if (result != 0) {
      if (this->isDebug()) {
         Serial.print("writeCommand: error writing command 0x");
         Serial.print(cbits, HEX);
         Serial.print(", result: ");
         Serial.println(result);
      }
      return false;
   } else
      return true;
}

bool cSHT3x::readResponse(std::uint8_t *buf, size_t nBuf) const {
   bool ok;
   unsigned nResult;
   const std::int8_t addr = this->getAddress();
   uint8_t nReadFrom;

   if (buf == nullptr || nBuf > 32 || addr < 0) {
      if (this->isDebug())
         Serial.println("readResponse: invalid parameter");

      return false;
   }

   nReadFrom = this->m_wire->requestFrom(std::uint8_t(addr), /* bytes */ std::uint8_t(nBuf));

   if (nReadFrom != nBuf) {
      if (this->isDebug()) {
         Serial.print("readResponse: nReadFrom(");
         Serial.print(unsigned(nReadFrom));
         Serial.print(") != nBuf(");
         Serial.print(nBuf);
         Serial.println(")");
      }
   }
   nResult = this->m_wire->available();

   for (unsigned i = 0; i < nResult; ++i)
      buf[i] = this->m_wire->read();

   if (nResult != nBuf && this->isDebug()) {
      Serial.print("readResponse: nResult(");
      Serial.print(nResult);
      Serial.print(") != nBuf(");
      Serial.print(nBuf);
      Serial.println(")");
   }

   return (nResult == nBuf);
}

std::uint8_t cSHT3x::crc(const std::uint8_t *buf, size_t nBuf, std::uint8_t crc8) {
   /* see CRC-8-Calc.md for a little info on this */
   static const std::uint8_t crcTable[16] =
       {
           0x00,
           0x31,
           0x62,
           0x53,
           0xc4,
           0xf5,
           0xa6,
           0x97,
           0xb9,
           0x88,
           0xdb,
           0xea,
           0x7d,
           0x4c,
           0x1f,
           0x2e,
       };

   for (size_t i = nBuf; i > 0; --i, ++buf) {
      uint8_t b, p;

      // calculate first nibble
      b = *buf;
      p = (b ^ crc8) >> 4;
      crc8 = (crc8 << 4) ^ crcTable[p];

      // calculate second nibble
      // this could be written as:
      //      b <<= 4;
      //      p = (b ^ crc8) >> 4;
      // but it's more effective as:
      p = ((crc8 >> 4) ^ b) & 0xF;
      crc8 = (crc8 << 4) ^ crcTable[p];
   }

   return crc8;
}

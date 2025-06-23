#ifndef _SHT3X_H_
#define _SHT3X_H_
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <cstdint>

namespace McciCatenaSht3x {

// create a version number for comparison
static constexpr std::uint32_t
makeVersion(
    std::uint8_t major, std::uint8_t minor, std::uint8_t patch, std::uint8_t local = 0) {
   return ((std::uint32_t)major << 24u) | ((std::uint32_t)minor << 16u) | ((std::uint32_t)patch << 8u) | (std::uint32_t)local;
}

// extract major number from version
static constexpr std::uint8_t
getMajor(std::uint32_t v) {
   return std::uint8_t(v >> 24u);
}

// extract minor number from version
static constexpr std::uint8_t
getMinor(std::uint32_t v) {
   return std::uint8_t(v >> 16u);
}

// extract patch number from version
static constexpr std::uint8_t
getPatch(std::uint32_t v) {
   return std::uint8_t(v >> 8u);
}

// extract local number from version
static constexpr std::uint8_t
getLocal(std::uint32_t v) {
   return std::uint8_t(v);
}

// version of library, for use by clients in static_asserts
static constexpr std::uint32_t kVersion = makeVersion(0, 2, 1, 0);

class cSHT3x {
  private:
   static constexpr bool kfDebug = false;

  public:
   // the address type:
   enum class Address_t : std::int8_t {
      Error = -1,
      A = 0x44,
      B = 0x45,
   };

   using Pin_t = std::int8_t;

   // constructor:
   cSHT3x(TwoWire &wire, Address_t Address = Address_t::A,
          Pin_t pinAlert = -1,
          Pin_t pinReset = -1)
       : m_wire(&wire),
         m_address(Address),
         m_pinAlert(pinAlert),
         m_pinReset(pinReset) {}

   // neither copyable nor movable
   cSHT3x(const cSHT3x &) = delete;
   cSHT3x &operator=(const cSHT3x &) = delete;
   cSHT3x(const cSHT3x &&) = delete;
   cSHT3x &operator=(const cSHT3x &&) = delete;

   static constexpr float rawTtoCelsius(std::uint16_t tfrac) {
      return -45.0f + 175.0f * (tfrac / 65535.0f);
   }

   static constexpr float rawRHtoPercent(std::uint16_t rhfrac) {
      return 100.0f * (rhfrac / 65535.0f);
   }

   static inline std::uint16_t celsiusToRawT(float t) {
      t += 45.0f;
      if (t < 0.0f)
         return 0;
      else if (t > 175.0)
         return 0xFFFFu;
      else
         return (std::uint16_t)((t / 175.0f) * 65535.0f);
   }

   static inline std::uint16_t percentRHtoRaw(float rh) {
      if (rh > 100.0)
         return 0xFFFFu;
      else if (rh < 0.0)
         return 0;
      else
         return (std::uint16_t)(65535.0f * (rh / 100.0));
   }

   // raw measurements as a collection.
   struct MeasurementsRaw {
      std::uint16_t TemperatureBits;
      std::uint16_t HumidityBits;
      void extract(std::uint16_t &a_t, std::uint16_t &a_rh) const {
         a_t = this->TemperatureBits;
         a_rh = this->HumidityBits;
      }
   };

   // measurements, as a collection.
   struct Measurements {
      float Temperature;
      float Humidity;
      void set(const MeasurementsRaw &mRaw) {
         this->Temperature = rawTtoCelsius(mRaw.TemperatureBits);
         this->Humidity = rawRHtoPercent(mRaw.HumidityBits);
      }
      void extract(float &a_t, float &a_rh) const {
         a_t = this->Temperature;
         a_rh = this->Humidity;
      }
   };

   // the commands -- not a class.
   enum class Command : std::uint16_t {
      // sorted in ascending numerical order.
      Error = 0,
      ModePeriodic_Medium_HalfHz = 0x2024,
      ModePeriodic_Low_HalfHz = 0x202F,
      ModePeriodic_High_HalfHz = 0x2032,
      ModePeriodic_Medium_1Hz = 0x2126,
      ModePeriodic_Low_1Hz = 0x212D,
      ModePeriodic_High_1Hz = 0x2130,
      ModePeriodic_Medium_2Hz = 0x2220,
      ModePeriodic_Low_2Hz = 0x222B,
      ModePeriodic_High_2Hz = 0x2236,
      ModePeriodic_Medium_4Hz = 0x2322,
      ModePeriodic_Low_4Hz = 0x2329,
      ModePeriodic_High_4Hz = 0x2334,
      ModeSingle_High_Nack = 0x2400,
      ModeSingle_Medium_Nack = 0x240B,
      ModeSingle_Low_Nack = 0x2416,
      ModePeriodic_Medium_10Hz = 0x2721,
      ModePeriodic_Low_10Hz = 0x272A,
      ModePeriodic_High_10Hz = 0x2737,
      ModePeriodic_ART = 0x2B32,
      ModeSingle_High_Stretch = 0x2C06,
      ModeSingle_Medium_Stretch = 0x2C0D,
      ModeSingle_Low_Stretch = 0x2C10,
      ClearStatus = 0x3041,
      HeaterDisable = 0x3066,
      HeaterEnable = 0x306D,
      Break = 0x3093,
      SoftReset = 0x30A2,
      Fetch = 0xE000,
      GetStatus = 0xF32D,
   };

   enum class ClockStretching : std::uint8_t {
      Disabled,
      Enabled,
   };
   enum class Periodicity : std::int8_t {
      Error = -1,
      Single,
      ART,
      HzHalf,
      HzOne,
      HzTwo,
      HzFour,
      HzTen
   };
   enum class Repeatability : std::int8_t {
      Error = -1,
      NA,
      Low,
      Medium,
      High,
   };

   static inline Command getCommand(Periodicity p, Repeatability r, ClockStretching s = ClockStretching::Disabled) {
      switch (p) {
         case Periodicity::Single:
            switch (r) {
               case Repeatability::Low:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModeSingle_Low_Nack
                             : Command::ModeSingle_Low_Stretch;
               case Repeatability::Medium:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModeSingle_Medium_Nack
                             : Command::ModeSingle_Medium_Stretch;
               case Repeatability::High:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModeSingle_High_Nack
                             : Command::ModeSingle_High_Stretch;
               default:
                  return Command::Error;
            }
            break;

         case Periodicity::ART:
            if (r == Repeatability::NA && p == Periodicity::HzFour && s == ClockStretching::Disabled)
               return Command::ModePeriodic_ART;
            else
               return Command::Error;
            break;

         case Periodicity::HzHalf:
            switch (r) {
               case Repeatability::Low:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Low_HalfHz
                             : Command::Error;
               case Repeatability::Medium:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Medium_HalfHz
                             : Command::Error;
               case Repeatability::High:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_High_HalfHz
                             : Command::Error;
               default:
                  return Command::Error;
            }
            break;

         case Periodicity::HzOne:
            switch (r) {
               case Repeatability::Low:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Low_1Hz
                             : Command::Error;
               case Repeatability::Medium:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Medium_1Hz
                             : Command::Error;
               case Repeatability::High:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_High_1Hz
                             : Command::Error;
               default:
                  return Command::Error;
            }
            break;

         case Periodicity::HzTwo:
            switch (r) {
               case Repeatability::Low:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Low_2Hz
                             : Command::Error;
               case Repeatability::Medium:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Medium_2Hz
                             : Command::Error;
               case Repeatability::High:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_High_2Hz
                             : Command::Error;
               default:
                  return Command::Error;
            }
            break;

         case Periodicity::HzFour:
            switch (r) {
               case Repeatability::Low:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Low_4Hz
                             : Command::Error;
               case Repeatability::Medium:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Medium_4Hz
                             : Command::Error;
               case Repeatability::High:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_High_4Hz
                             : Command::Error;
               default:
                  return Command::Error;
            }
            break;

         case Periodicity::HzTen:
            switch (r) {
               case Repeatability::Low:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Low_10Hz
                             : Command::Error;
               case Repeatability::Medium:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_Medium_10Hz
                             : Command::Error;
               case Repeatability::High:
                  return (s == ClockStretching::Disabled)
                             ? Command::ModePeriodic_High_10Hz
                             : Command::Error;
               default:
                  return Command::Error;
            }
            break;

         default:
            return Command::Error;
      }
   }

   static inline Periodicity getPeriodicity(Command c) {
      switch (c) {
         case Command::ModePeriodic_ART:
            return Periodicity::ART;
         case Command::ModeSingle_High_Nack:
         case Command::ModeSingle_High_Stretch:
         case Command::ModeSingle_Medium_Nack:
         case Command::ModeSingle_Medium_Stretch:
         case Command::ModeSingle_Low_Nack:
         case Command::ModeSingle_Low_Stretch:
            return Periodicity::Single;
         case Command::ModePeriodic_Low_HalfHz:
         case Command::ModePeriodic_Medium_HalfHz:
         case Command::ModePeriodic_High_HalfHz:
            return Periodicity::HzHalf;
         case Command::ModePeriodic_Low_1Hz:
         case Command::ModePeriodic_Medium_1Hz:
         case Command::ModePeriodic_High_1Hz:
            return Periodicity::HzOne;
         case Command::ModePeriodic_Low_2Hz:
         case Command::ModePeriodic_Medium_2Hz:
         case Command::ModePeriodic_High_2Hz:
            return Periodicity::HzTwo;
         case Command::ModePeriodic_Low_4Hz:
         case Command::ModePeriodic_Medium_4Hz:
         case Command::ModePeriodic_High_4Hz:
            return Periodicity::HzFour;
         case Command::ModePeriodic_Low_10Hz:
         case Command::ModePeriodic_Medium_10Hz:
         case Command::ModePeriodic_High_10Hz:
            return Periodicity::HzTen;
         default:
            return Periodicity::Error;
      }
   }

   static inline ClockStretching getClockStretching(Command c) {
      switch (c) {
         case Command::ModeSingle_High_Stretch:
         case Command::ModeSingle_Medium_Stretch:
         case Command::ModeSingle_Low_Stretch:
            return ClockStretching::Enabled;
         default:
            return ClockStretching::Disabled;
      }
   }

   static inline Repeatability getRepeatability(Command c) {
      switch (c) {
         case Command::ModePeriodic_ART:
            return Repeatability::NA;

         case Command::ModeSingle_High_Nack:
         case Command::ModeSingle_High_Stretch:
         case Command::ModePeriodic_High_HalfHz:
         case Command::ModePeriodic_High_1Hz:
         case Command::ModePeriodic_High_2Hz:
         case Command::ModePeriodic_High_4Hz:
         case Command::ModePeriodic_High_10Hz:
            return Repeatability::High;

         case Command::ModeSingle_Medium_Nack:
         case Command::ModeSingle_Medium_Stretch:
         case Command::ModePeriodic_Medium_HalfHz:
         case Command::ModePeriodic_Medium_1Hz:
         case Command::ModePeriodic_Medium_2Hz:
         case Command::ModePeriodic_Medium_4Hz:
         case Command::ModePeriodic_Medium_10Hz:
            return Repeatability::Medium;

         case Command::ModeSingle_Low_Nack:
         case Command::ModeSingle_Low_Stretch:
         case Command::ModePeriodic_Low_HalfHz:
         case Command::ModePeriodic_Low_1Hz:
         case Command::ModePeriodic_Low_2Hz:
         case Command::ModePeriodic_Low_4Hz:
         case Command::ModePeriodic_Low_10Hz:
            return Repeatability::Low;
         default:
            return Repeatability::Error;
      }
   }

   // return the highest periodicity value such that
   //  1. PeriodicityToMillis(millisToPeriodicity(ms)) != 0
   //  2. PeriodicityToMillis(millisToPeriodicity(ms)) <= ms
   static inline Periodicity millisToPeriodicity(std::uint32_t ms) {
      if (ms < 250)
         return Periodicity::HzTen;
      else if (ms < 500)
         return Periodicity::HzFour;
      else if (ms < 1000)
         return Periodicity::HzTwo;
      else if (ms < 2000)
         return Periodicity::HzOne;
      else
         return Periodicity::HzHalf;
   }

   // return the ms/sample corresponding to p, or zero if not
   // a periodic value.
   static inline std::uint32_t PeriodicityToMillis(Periodicity p) {
      switch (p) {
         case Periodicity::HzHalf:
            return 2000;
         case Periodicity::HzOne:
            return 1000;
         case Periodicity::HzTwo:
            return 500;
         case Periodicity::ART:
         case Periodicity::HzFour:
            return 250;
         case Periodicity::HzTen:
            return 100;
         default:
            return 0;
      }
   }

   // status bits
   class Status_t {
     public:
      Status_t(std::uint32_t status = 1 << 16)
          : m_Status(status){};

      // copy constructor
      Status_t(const Status_t &a) : m_Status(a.m_Status) {}

      bool isAlert() const { return this->m_Status & (1 << 15); }
      bool isHeaterOn() const { return this->m_Status & (1 << 13); }
      bool isRHTrackingAlert() const { return this->m_Status & (1 << 11); }
      bool isTemperatureTrackingAlert() const { return this->m_Status & (1 << 10); }
      bool isSystemResetDetected() const { return this->m_Status & (1 << 4); }
      bool isCommandFailure() const { return this->m_Status & (1 << 1); }
      bool isCommandBadCS() const { return this->m_Status & (1 << 0); }
      bool isValid() const { return (this->m_Status & (1 << 16)) == 0; }
      std::uint16_t getBits() const { return this->m_Status & 0xFFFF; }

     private:
      std::uint32_t m_Status;
   };

   // start operation.
   bool begin();

   // end operation.
   void end();

   Status_t getStatus(void) const;

   bool getTemperatureHumidityRaw(std::uint16_t &t, std::uint16_t &rh, Repeatability r = Repeatability::High) const;
   bool getTemperatureHumidityRaw(MeasurementsRaw &mRaw, Repeatability r = Repeatability::High) const;
   bool getTemperatureHumidity(float &T, float &rh, Repeatability r = Repeatability::High) const;
   bool getTemperatureHumidity(Measurements &m, Repeatability r = Repeatability::High) const;
   bool reset(void) const;

   // start a measurement, and return the millis to delay between
   // measurements
   std::uint32_t startPeriodicMeasurement(Command c) const;
   bool getPeriodicMeasurement(float &T, float &rh) const;
   bool getPeriodicMeasurement(Measurements &m) const;
   bool getPeriodicMeasurementRaw(std::uint16_t &tfrac, std::uint16_t &rhfrac) const;
   bool getPeriodicMeasurementRaw(MeasurementsRaw &mRaw) const;

   bool setCrcMode(bool newMode) {
      bool const oldMode = !this->m_noCrc;
      this->m_noCrc = !newMode;
      return oldMode;
   }

   bool getCrcMode() const { return !this->m_noCrc; }

   bool setHeater(bool fOn) const {
      return this->writeCommand(
          fOn ? Command::HeaterEnable
              : Command::HeaterDisable);
   }

   bool getHeater(void) const;

   static constexpr bool isDebug() { return kfDebug; }

  protected:
   bool writeCommand(Command c) const;
   bool readResponse(std::uint8_t *buf, size_t nBuf) const;
   bool processResultsRaw(const std::uint8_t (&buf)[6], std::uint16_t &t, std::uint16_t &rh) const;
   bool processResultsRaw(const std::uint8_t (&buf)[6], MeasurementsRaw &mRaw) const;
   static std::uint8_t crc(const std::uint8_t *buf, size_t nBuf, std::uint8_t crc8 = 0xFF);
   std::int8_t getAddress() const { return static_cast<std::int8_t>(this->m_address); }

  private:
   TwoWire *m_wire;
   Address_t m_address;
   Pin_t m_pinAlert;
   Pin_t m_pinReset;
   bool m_noCrc;
};

}

#endif /* undef(_SHT3X_H_) */

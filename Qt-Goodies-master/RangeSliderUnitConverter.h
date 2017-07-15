#ifndef RANGESLIDERUNITCONVERTER_H
#define RANGESLIDERUNITCONVERTER_H

/**
 * Used for converting native units, to some logical unit
 */
class RangeSliderUnitConverter {
 public:
  /**
   * Destructs the RangeSliderUnitConverter
   */
  virtual ~RangeSliderUnitConverter() {}

  /**
   * Convert from the base unit to logical unit
   *
   * @param baseunit The unit of the base
   */
  virtual QVariant convertFromBase(int baseunit) const = 0;

  /**
   * Converts from a logical unit to int
   *
   * @param logical Some representation that can be converted to an int
   */
  virtual int convertToBase(QVariant logical) const = 0;
};

#endif

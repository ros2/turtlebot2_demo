/**
 * @file /ecl_utilities/include/ecl/utilities/flags.hpp
 *
 * Flags c++ style without all the bit operations.
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_UTILITIES_FLAGS_HPP_
#define ECL_UTILITIES_FLAGS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{

/*****************************************************************************
 ** Classes
 *****************************************************************************/

enum Bits
{
  Bit0 =  0x0000, Bit1 =  0x0001, Bit2 =  0x0002, Bit3 =  0x0004, Bit4 =  0x0008, Bit5 =  0x0010, Bit6 =  0x0020,
  Bit7 =  0x0040, Bit8 =  0x0080, Bit9 =  0x0100, Bit10 = 0x0200, Bit11 = 0x0400, Bit12 = 0x0800, Bit13 = 0x1000,
  Bit14 = 0x2000, Bit15 = 0x4000, Bit16 = 0x8000,
};
/**
 * @brief  Convenience class for organising boolean flags.
 *
 * This class organises a group of flags (via enums) in a convenient
 * and typesafe manner. It is essentially a container for flags (in the form
 * of enums) with a convenient interface on top.
 *
 * Usage:
 *
 * - Define an appropriate bit-arithmetic enum.
 *
 * @code
 * enum Settings {
 *     Fast   0x0001,
 *     Medium 0x0002,
 *     Slow   0x0004,
 *     Red    0x0010,
 *     Blue   0x0020
 * }
 * @endcode
 *
 * - Pipe flags into the enum
 *
 * @code
 * settings = settings|Fast|Slow;
 * @endcode
 *
 * - Set up a friend operator on the enum type for more convenient piping (see the dox).
 * - Use one of the variety of mask/compliment/access operators for handling (see method documentation).
 **/
template<typename Enum>
  class Flags
  {
  public:
    /*********************
     ** C&D
     **********************/
    Flags() : value(0) {}; /**< Default constructor. **/
    Flags(const Flags<Enum> & other) : value(other.value) {}; /**< Copy constructor. **/
    Flags(Enum flag) : value(flag) {}; /**< Raw constructor. **/
    ~Flags() { }
    ;

    /*********************
     ** Utility
     **********************/
    void clear() { value = 0; } /**< Clear all flags. **/
    /**
     * Test current status of a flag.
     **/
    bool testFlag(Enum flag) const
    {
      if ((value & flag) != 0)
      {
        return true;
      }
      else
      {
        if ( ( flag == 0 ) && ( value == 0 ) ) {
          return true;
        } else {
          return false;
        }
      }
    }
    ;

    /*********************
     ** Type compatibility
     **********************/
    operator int() const
    {
      return value;
    }
    ; /**< Type compatibility with ints. **/

    /******************************************
     ** Return new instance
     *******************************************/
    /**
     * Mask operator.
     * @return Flags<Enum> : new instance of the flag set.
     **/
    Flags<Enum> operator&(int mask) const
    {
      Flags<Enum> f;
      f.value = value & mask;
      return f;
    }
    ;
    /**
     * Single flag masking operator.
     * @return Flags<Enum> : new instance of the flag set.
     **/
    Flags<Enum> operator&(Enum flag) const
    {
      Flags<Enum> f;
      f.value = value & flag;
      return f;
    }
    ;
    /**
     * OR operator.
     * @return Flags<Enum> : new instance of the flag set.
     **/
    Flags<Enum> operator|(Flags<Enum> other) const
    {
      Flags<Enum> f;
      f.value = value | other.value;
      return f;
    }
    ;
    /**
     * Single flag OR operator.
     * @return Flags<Enum> : new instance of the flag set.
     **/
    Flags<Enum> operator|(Enum flag) const
    {
      Flags<Enum> f;
      f.value = value | flag;
      return f;
    }
    ;
    /**
     * XOR operator.
     * @return Flags<Enum> : new instance of the flag set.
     **/
    Flags<Enum> operator^(Flags<Enum> other) const
    {
      Flags<Enum> f;
      f.value = value ^ other.value;
      return f;
    }
    ;
    /**
     * Single flag XOR operator.
     * @return Flags<Enum> : new instance of the flag set.
     **/
    Flags<Enum> operator^(Enum flag) const
    {
      Flags<Enum> f;
      f.value = value ^ flag;
      return f;
    }
    ;
    /**
     * Complement operator.
     * @return Flags<Enum> : new instance of the flag set.
     **/
    Flags<Enum> operator~() const
    {
      Flags<Enum> f;
      f.value = ~value;
      return f;
    }
    ;

    /******************************************
     ** Update current instance
     *******************************************/
    /**
     * Assignment operator.
     * @return Flags<Enum> : updated instance of the flag set.
     **/
    Flags<Enum> & operator=(const Flags<Enum> & other)
    {
      value = other.value;
      return *this;
    }
    ;
    /**
     * Assignment operator.
     * @return Flags<Enum> : updated instance of the flag set.
     **/
    Flags<Enum> & operator=(const Enum & flag)
    {
      value = static_cast<int>(flag);
      return *this;
    }
    ;
    /**
     * Self operating mask.
     * @return Flags<Enum> : updated instance of the masked flag set.
     **/
    Flags<Enum> & operator&=(int mask)
    {
      value &= mask;
      return *this;
    }
    ;
    /**
     * Self operating OR operation.
     * @return Flags<Enum> : updated instance of the masked flag set.
     **/
    Flags<Enum> & operator|=(Flags<Enum> other)
    {
      value |= other;
      return *this;
    }
    ;
    /**
     * Self operating OR operation with a single flag.
     * @return Flags<Enum> : updated instance of the masked flag set.
     **/
    Flags<Enum> & operator|=(Enum flag)
    {
      value |= flag;
      return *this;
    }
    ;
    /**
     * Self operating XOR operation.
     * @return Flags<Enum> : updated instance of the masked flag set.
     **/
    Flags<Enum> & operator^=(Flags<Enum> other)
    {
      value ^= other;
      return *this;
    }
    ;
    /**
     * Self operating XOR operation with a single flag.
     * @return Flags<Enum> : updated instance of the masked flag set.
     **/
    Flags<Enum> & operator^=(Enum flag)
    {
      value ^= flag;
      return *this;
    }
    ;

    /*********************
     ** Helper Methods
     **********************/
    /**
     * Adding a flag (OR operation) to an existing flag set.
     * @return Flags<Enum> : resulting instance of the flag combination.
     **/
    friend inline Flags<Enum> operator|(Enum flag, Flags<Enum> flags)
    {
      return flags | flag;
    }
  private:
    int value;
  };

} // namespace ecl

#endif /* ECL_UTILITIES_FLAGS_HPP_ */

/**
 * @file include/kobuki_driver/packet_handler/payload_base.hpp
 *
 * @brief Base class for payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ROBOT_DATA_HPP_
#define ROBOT_DATA_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ecl/containers.hpp>
#include <stdint.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace packet_handler
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief
 * Provides base class for payloads.
 *
 */
class payloadBase
{
public:

  /**
   * this is simple magic to write the flag, when we get the packet from the host or
   * when we want to send the data
   */
  bool yes;

  /**
   * it indicates the type of derived packet. if packet type is dynamic, length of
   * packet can be changed during communication session. Ohterwise can not.
   */
  const bool is_dynamic;

  /**
   * it indicates length of data part of packet, except header and length field.
   * if packet is fixed type, this value should be matched with length field.
   * if packet is dynamic type, this value indicates minimal value of length field.
   */
  const unsigned char length;

  /*
   * construct and destruct
   */
  payloadBase(const bool is_dynamic_ = false, const unsigned char length_ = 0 )
    : yes(false)
    , is_dynamic(is_dynamic_)
    , length(length_)
  {};
  virtual ~payloadBase() {};

  /*
   * serialisation
   */
  virtual bool serialise(ecl::PushAndPop<unsigned char> & byteStream)=0;
  virtual bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)=0;

  // utilities
  // todo; let's put more useful converters here. Or we may use generic converters
protected:
  // below funciton should be replaced wiht converter
  template<typename T>
    void buildVariable(T & V, ecl::PushAndPop<unsigned char> & buffer)
    {
      if (buffer.size() < sizeof(T))
        return;
      V = static_cast<unsigned char>(buffer.pop_front());

      unsigned int size_value(sizeof(T));
      for (unsigned int i = 1; i < size_value; i++)
      {
        V |= ((static_cast<unsigned char>(buffer.pop_front())) << (8 * i));
      }
    }

  template<typename T>
    void buildBytes(const T & V, ecl::PushAndPop<unsigned char> & buffer)
    {
      unsigned int size_value(sizeof(T));
      for (unsigned int i = 0; i < size_value; i++)
      {
        buffer.push_back(static_cast<unsigned char>((V >> (i * 8)) & 0xff));
      }
    }
};

/**
 * Need to be very careful with this - it will only work across platforms if they
 * happen to be doing reinterpret_cast with the same float standard.
 * @param V
 * @param buffer
 */
template<>
inline   void payloadBase::buildVariable<float>(float & V, ecl::PushAndPop<unsigned char> & buffer)
  {
    if (buffer.size() < 4)
      return;
    unsigned int ui;
    ui = static_cast<unsigned char>(buffer.pop_front());

    unsigned int size_value(4);
    for (unsigned int i = 1; i < size_value; i++)
    {
      ui |= ((static_cast<unsigned char>(buffer.pop_front())) << (8 * i));
    }

    V = reinterpret_cast<float&>(ui);
  }

template<>
inline void payloadBase::buildBytes<float>(const float & V, ecl::PushAndPop<unsigned char> & buffer)
  {
    if (buffer.size() < 4)
      return;
    unsigned int size_value(4);
    unsigned int ui(reinterpret_cast<const unsigned int&>(V));
    for (unsigned int i = 0; i < size_value; i++)
    {
      buffer.push_back(static_cast<unsigned char>((ui >> (i * 8)) & 0xff));
    }
  }

//#define FRAC_MAX 9223372036854775807LL /* 2**63 - 1 */
//
//struct dbl_packed
//{
//    int exp;
//    long long frac;
//};
//
//void pack(double x, struct dbl_packed *r)
//{
//    double xf = fabs(frexp(x, &r->exp)) - 0.5;
//
//    if (xf < 0.0)
//    {
//        r->frac = 0;
//        return;
//    }
//
//    r->frac = 1 + (long long)(xf * 2.0 * (FRAC_MAX - 1));
//
//    if (x < 0.0)
//        r->frac = -r->frac;
//}
//
//double unpack(const struct dbl_packed *p)
//{
//    double xf, x;
//
//    if (p->frac == 0)
//        return 0.0;
//
//    xf = ((double)(llabs(p->frac) - 1) / (FRAC_MAX - 1)) / 2.0;
//
//    x = ldexp(xf + 0.5, p->exp);
//
//    if (p->frac < 0)
//        x = -x;
//
//    return x;
//}

}
;
// namespace packet_handler

#endif /* ROBOT_DATA_HPP_ */

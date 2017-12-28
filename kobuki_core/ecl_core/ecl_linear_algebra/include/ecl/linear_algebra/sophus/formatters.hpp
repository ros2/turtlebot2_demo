/**
 * @file /ecl_linear_algebra/include/ecl_linear_algebra/sophus_formatters.hpp
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ecl_linear_algebra_SOPHUS_FORMATTERS_HPP_
#define ecl_linear_algebra_SOPHUS_FORMATTERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/exceptions.hpp>
#include <ecl/formatters.hpp>
#include <sophus/se3.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Sophus {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * Formats the se3 in easily readable translation/quaternion syntax.
 *
 * If you are working with frames, what you usually want to read is the
 * pose of the frame, i.e. the inverse of the transform.
 *
 * e.g. if you have T_cam_rel_map, to get the pose of the cam frame relative
 * to the map frame, pass T_cam_rel_map.inverse() to this formatter.
 */
class SE3fFormatter
{
public:
  /**
   * @brief Default constructor.
   *
   * Initialises the format tags for width, and precision.
   * @param w : width (default - no width constraints)
   * @param p : the number of decimal places of precision (default - 4)
   **/
  SE3fFormatter(const int &w = -1, const unsigned int &p = 2)
  : format(w, p, ecl::RightAlign)
  , tmp_width(w)
  , tmp_precision(p)
  , tmp_formatting(false)
  , ready_to_format(false)
  , s_(NULL)
{}
  virtual ~SE3fFormatter() {}
  /**
   * @brief Sets the precision format parameter.
   *
   * @param p : the number of decimal places of precision.
   * @return SE3fFormatter& : this formatter readied for use with a stream.
   **/
  SE3fFormatter& precision( const unsigned int &p ) {
    format.precision(p);
    return *this;
  }
  /**
   * @brief Sets the width format parameter.
   *
   * Sets the width format parameter.
   *
   * @param w : the width to use for inserted floats (-1 is no width constraint).
   * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
   **/
  SE3fFormatter& width( const int &w ) {
    format.width(w);
    return *this;
  }
  /**
   * @brief Returns the current precision setting.
   * @return unsigned int : the precision value.
   */
  unsigned int precision() { return format.precision(); }

  /**
   * @brief Returns the current width setting.
   * @return int : the witdh value (-1 for no width constraint).
   */
  int width() { return format.width(); }

  /**
   * @brief Format a sophus transform with permanently stored precision/width.
   *
   * This function directly formats the specified input value
   * with the stored settings.
   * @code
   * cout << format(Sophus::SE3f()) << endl;
   * @endcode
   * @param matrix : the matrix to be formatted (gets temporarily stored as a pointer).
   * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
   **/
  SE3fFormatter& operator() (const Sophus::SE3f & s ) {
    s_ = &s;
    ready_to_format = true;
    return (*this);
  }
  /**
   * @brief Format a matrix with temporarily specified precision/width.
   *
   * This function directly formats the specified input value
   * and temporary settings.
   *
   * @code
   * Sophus::SE3f s;
   * cout << format(s,3,-1) << endl; // precision of 3 and no width constraint
   * @endcode
   *
   * @param matrix : the matrix to be formatted (gets temporarily stored as a pointer).
   * @param w : the width to use for inserted floats (-1 is no width constraint).
   * @param p : the number of decimal places of precision.
   * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
   **/
  SE3fFormatter& operator() (const Sophus::SE3f & s, const int &w, const unsigned int &p) {
          s_ = &s;
          tmp_precision = p;
          tmp_width = w;
          tmp_formatting = true;
          ready_to_format = true;
          return (*this);
  }
  /**
   * @brief Stream the formatter.
   *
   * Insertion operator for sending the formatter  to an output stream.
   *
   * @param ostream : the output stream.
   * @param formatter : the formatter to be inserted.
   * @tparam OutputStream : the type of the output stream to be inserted into.
   * @tparam Derived : matrix type.
   * @return OutputStream : continue streaming with the updated output stream.
   *
   * @exception StandardException : throws if the formatter has un-specified _matrix [debug mode only]
   */
  template <typename OutputStream>
  friend OutputStream& operator << ( OutputStream& ostream, SE3fFormatter & formatter ) ecl_assert_throw_decl(ecl::StandardException);

private:
  ecl::Format<float> format;
  int tmp_width;
  unsigned int tmp_precision;
  bool tmp_formatting;
  bool ready_to_format;
  const Sophus::SE3f *s_;
};

template <typename OutputStream>
OutputStream& operator << (OutputStream& ostream, SE3fFormatter & formatter ) ecl_assert_throw_decl(ecl::StandardException) {

  ecl_assert_throw( formatter.s_, ecl::StandardException(LOC,ecl::UsageError,"The formatter cannot print any data - "
          "sophus object was not initialised "
          "please pass the your matrix through () operator") );

  ecl_assert_throw(formatter.ready_to_format, ecl::StandardException(LOC,ecl::UsageError,"The formatter cannot print any data - "
          "either there is no data available, or you have tried to use the "
          "formatter more than once in a single streaming operation. "
          "C++ produces unspecified results when functors are used multiply "
          "in the same stream sequence, so this is not permitted here.") );

  if ( formatter.ready_to_format ) {
    unsigned int prm_precision = formatter.format.precision();;
    int prm_width = formatter.format.width();
    if ( formatter.tmp_formatting ) {
      formatter.format.precision(formatter.tmp_precision);
      formatter.format.width(formatter.tmp_width);
    }

    /*********************
    ** Stream Sophus
    **********************/
    Eigen::Vector3f translation = formatter.s_->translation();
    Eigen::Quaternionf quaternion = formatter.s_->unit_quaternion();
    ostream << "[";
    ostream <<  "x:" << formatter.format(translation.x()) << "  ";
    ostream <<  "y:" << formatter.format(translation.y()) << "  ";
    ostream <<  "z:" << formatter.format(translation.z());
    ostream << "]";
    ostream << "[";
    ostream <<  "x:" << formatter.format(quaternion.x()) << "  ";
    ostream <<  "y:" << formatter.format(quaternion.y()) << "  ";
    ostream <<  "z:" << formatter.format(quaternion.z()) << "  ";
    ostream <<  "w:" << formatter.format(quaternion.w()) << "  ";
    ostream << "]";

    if ( formatter.tmp_formatting ) {
      formatter.format.precision(prm_precision);
      formatter.format.width(prm_width);
      formatter.tmp_formatting = false;
    }
    formatter.ready_to_format = false;
  }
  return ostream;
}

} // namespace Sophus

/*****************************************************************************
 ** ECL Format Type
 *****************************************************************************/

namespace ecl {

  template <>
  class Format<Sophus::SE3f> : public Sophus::SE3fFormatter {};

} // namespace ecl

#endif /* ecl_linear_algebra_SOPHUS_FORMATTERS_HPP_ */

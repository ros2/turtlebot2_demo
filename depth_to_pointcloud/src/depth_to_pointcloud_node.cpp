#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_geometry/pinhole_camera_model.h>

static rclcpp::publisher::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud;

static sensor_msgs::msg::CameraInfo::SharedPtr cam_info;

/** Private base class for PointCloud2Iterator and PointCloud2ConstIterator
 * T is the type of the value on which the child class will be templated
 * TT is the type of the value to be retrieved (same as T except for constness)
 * U is the type of the raw data in PointCloud2 (only uchar and const uchar are supported)
 * C is the type of the pointcloud to intialize from (const or not)
 * V is the derived class (yop, curiously recurring template pattern)
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
class PointCloud2IteratorBase
{
public:
  /**
   */
  PointCloud2IteratorBase();

  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  PointCloud2IteratorBase(C &cloud_msg, const std::string &field_name);

  /** Assignment operator
   * @param iter the iterator to copy data from
   * @return a reference to *this
   */
  V<T>& operator =(const V<T>& iter);

  /** Access the i th element starting at the current pointer (useful when a field has several elements of the same
   * type)
   * @param i
   * @return a reference to the i^th value from the current position
   */
  TT& operator [](size_t i) const;

  /** Dereference the iterator. Equivalent to accessing it through [0]
   * @return the value to which the iterator is pointing
   */
  TT& operator *() const;

  /** Increase the iterator to the next element
   * @return a reference to the updated iterator
   */
  V<T>& operator ++();

  /** Basic pointer addition
   * @param i the amount to increase the iterator by
   * @return an iterator with an increased position
   */
  V<T> operator +(int i);

  /** Increase the iterator by a certain amount
   * @return a reference to the updated iterator
   */
  V<T>& operator +=(int i);

  /** Compare to another iterator
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator !=(const V<T>& iter) const;

  /** Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++)
   */
  V<T> end() const;

private:
  /** Common code to set the field of the PointCloud2
   * @param cloud_msg the PointCloud2 to modify
   * @param field_name the name of the field to iterate upon
   * @return the offset at which the field is found
   */
  int set_field(const sensor_msgs::msg::PointCloud2 &cloud_msg, const std::string &field_name);

  /** The "point_step" of the original cloud */
  int point_step_;
  /** The raw data  in uchar* where the iterator is */
  U* data_char_;
  /** The cast data where the iterator is */
  TT* data_;
  /** The end() pointer of the iterator */
  TT* data_end_;
  /** Whether the fields are stored as bigendian */
  bool is_bigendian_;
};

/**
 * \brief Class that can iterate over a PointCloud2
 *
 * T type of the element being iterated upon
 * E.g, you create your PointClou2 message as follows:
 * <PRE>
 *   setPointCloud2FieldsByString(cloud_msg, 2, "xyz", "rgb");
 * </PRE>
 *
 * For iterating over XYZ, you do :
 * <PRE>
 *   sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
 * </PRE>
 * and then access X through iter_x[0] or *iter_x
 * You could create an iterator for Y and Z too but as they are consecutive,
 * you can just use iter_x[1] and iter_x[2]
 *
 * For iterating over RGB, you do:
 * <PRE>
 * sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_msg, "rgb");
 * </PRE>
 * and then access R,G,B through  iter_rgb[0], iter_rgb[1], iter_rgb[2]
 */
template<typename T>
class PointCloud2Iterator : public PointCloud2IteratorBase<T, T, unsigned char, sensor_msgs::msg::PointCloud2, PointCloud2Iterator>
{
public:
  PointCloud2Iterator(sensor_msgs::msg::PointCloud2 &cloud_msg, const std::string &field_name) :
    PointCloud2IteratorBase<T, T, unsigned char, sensor_msgs::msg::PointCloud2, PointCloud2Iterator>::PointCloud2IteratorBase(cloud_msg, field_name) {}
};

/**
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, C, V>::PointCloud2IteratorBase() : data_char_(0), data_(0), data_end_(0)
{
}

/**
 * @param cloud_msg_ The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, C, V>::PointCloud2IteratorBase(C &cloud_msg, const std::string &field_name)
{
  int offset = set_field(cloud_msg, field_name);

  data_char_ = &(cloud_msg.data.front()) + offset;
  data_ = reinterpret_cast<TT*>(data_char_);
  data_end_ = reinterpret_cast<TT*>(&(cloud_msg.data.back()) + 1 + offset);
}

/** Assignment operator
 * @param iter the iterator to copy data from
 * @return a reference to *this
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator =(const V<T> &iter)
{
  if (this != &iter)
  {
    point_step_ = iter.point_step_;
    data_char_ = iter.data_char_;
    data_ = iter.data_;
    data_end_ = iter.data_end_;
    is_bigendian_ = iter.is_bigendian_;
  }

  return *this;
}

/** Access the i th element starting at the current pointer (useful when a field has several elements of the same
 * type)
 * @param i
 * @return a reference to the i^th value from the current position
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, C, V>::operator [](size_t i) const
{
  return *(data_ + i);
}

/** Dereference the iterator. Equivalent to accessing it through [0]
 * @return the value to which the iterator is pointing
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, C, V>::operator *() const
{
  return *data_;
}

/** Increase the iterator to the next element
 * @return a reference to the updated iterator
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator ++()
{
  data_char_ += point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}

/** Basic pointer addition
 * @param i the amount to increase the iterator by
 * @return an iterator with an increased position
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, C, V>::operator +(int i)
{
  V<T> res = *static_cast<V<T>*>(this);

  res.data_char_ += i*point_step_;
  res.data_ = reinterpret_cast<TT*>(res.data_char_);

  return res;
}

/** Increase the iterator by a certain amount
 * @return a reference to the updated iterator
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator +=(int i)
{
  data_char_ += i*point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}

/** Compare to another iterator
 * @return whether the current iterator points to a different address than the other one
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
bool PointCloud2IteratorBase<T, TT, U, C, V>::operator !=(const V<T>& iter) const
{
  return iter.data_ != data_;
}

/** Return the end iterator
 * @return the end iterator (useful when performing normal iterator processing with ++)
 */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, C, V>::end() const
{
  V<T> res = *static_cast<const V<T>*>(this);
  res.data_ = data_end_;
  return res;
}

/** Common code to set the field of the PointCloud2
  * @param cloud_msg the PointCloud2 to modify
  * @param field_name the name of the field to iterate upon
  * @return the offset at which the field is found
  */
template<typename T, typename TT, typename U, typename C, template <typename> class V>
int PointCloud2IteratorBase<T, TT, U, C, V>::set_field(const sensor_msgs::msg::PointCloud2 &cloud_msg, const std::string &field_name)
{
  is_bigendian_ = cloud_msg.is_bigendian;
  point_step_ = cloud_msg.point_step;
  // make sure the channel is valid
  std::vector<sensor_msgs::msg::PointField>::const_iterator field_iter = cloud_msg.fields.begin(), field_end =
      cloud_msg.fields.end();
  while ((field_iter != field_end) && (field_iter->name != field_name))
    ++field_iter;

  if (field_iter == field_end) {
    // Handle the special case of r,g,b,a (we assume they are understood as the channels of an rgb or rgba field)
    if ((field_name == "r") || (field_name == "g") || (field_name == "b") || (field_name == "a"))
    {
      // Check that rgb or rgba is present
      field_iter = cloud_msg.fields.begin();
      while ((field_iter != field_end) && (field_iter->name != "rgb") && (field_iter->name != "rgba"))
        ++field_iter;
      if (field_iter == field_end)
        throw std::runtime_error("Field " + field_name + " does not exist");
      if (field_name == "r")
      {
        if (is_bigendian_)
          return field_iter->offset + 1;
        else
          return field_iter->offset + 2;
      }
      if (field_name == "g")
      {
        if (is_bigendian_)
          return field_iter->offset + 2;
        else
          return field_iter->offset + 1;
      }
      if (field_name == "b")
      {
        if (is_bigendian_)
          return field_iter->offset + 3;
        else
          return field_iter->offset + 0;
      }
      if (field_name == "a")
      {
        if (is_bigendian_)
          return field_iter->offset + 0;
        else
          return field_iter->offset + 3;
      }
    } else
      throw std::runtime_error("Field " + field_name + " does not exist");
  }

  return field_iter->offset;
}

// Encapsulate differences between processing float and uint16_t depths
template<typename T> struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) { return depth != 0; }
  static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
  static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
  static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(float depth) { return depth; }
  static inline float fromMeters(float depth) { return depth; }

  static inline void initializeBuffer(std::vector<uint8_t>& buffer)
  {
    float* start = reinterpret_cast<float*>(&buffer[0]);
    float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};

// Handles float or uint16 depths
template<typename T>
void convert(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
    const image_geometry::PinholeCameraModel& model,
    double range_max = 0.0)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        if (range_max != 0.0)
        {
          depth = DepthTraits<T>::fromMeters(range_max);
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
    }
  }
}

void depthCb(const sensor_msgs::msg::Image::SharedPtr image)
{
  if (cam_info == nullptr)
  {
    // we haven't gotten the camera info yet, so just drop until
    // we do.
    fprintf(stderr, "No camera info, skipping point cloud conversion\n");
    return;
  }
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header = image->header;
  cloud_msg->height = image->height;
  cloud_msg->width = image->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  cloud_msg->fields.clear();
  cloud_msg->fields.reserve(1);

  sensor_msgs::msg::PointField pfx;
  pfx.name = "x";
  pfx.count = 1;
  pfx.datatype = sensor_msgs::msg::PointField::FLOAT32;
  pfx.offset = 0;
  cloud_msg->fields.push_back(pfx);

  sensor_msgs::msg::PointField pfy;
  pfy.name = "y";
  pfy.count = 1;
  pfy.datatype = sensor_msgs::msg::PointField::FLOAT32;
  pfy.offset = 4;
  cloud_msg->fields.push_back(pfy);

  sensor_msgs::msg::PointField pfz;
  pfz.name = "z";
  pfz.count = 1;
  pfz.datatype = sensor_msgs::msg::PointField::FLOAT32;
  pfz.offset = 8;
  cloud_msg->fields.push_back(pfz);

  cloud_msg->point_step = 16;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);

  // info_msg here is a sensor_msg::msg::CameraInfo::ConstSharedPtr; we should be able to get this from the astra_driver layer.
  image_geometry::PinholeCameraModel model_;
  model_.fromCameraInfo(cam_info);

  convert<float>(image, cloud_msg, model_);

  cloud_msg->header.frame_id = std::string("openni_depth_optical_frame");
  g_pub_point_cloud->publish(cloud_msg);
}

void infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  cam_info = info;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::node::Node::SharedPtr node = rclcpp::node::Node::make_shared("depth_to_pointcloud");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.depth = 1;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

  g_pub_point_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("points2", custom_qos_profile);

  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>("depth", depthCb, custom_qos_profile);
  auto cam_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("depth_camera_info", infoCb, custom_qos_profile);

  rclcpp::spin(node);

  return 0;
}

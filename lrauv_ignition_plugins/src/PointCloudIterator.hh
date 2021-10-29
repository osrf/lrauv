// Copyright (c) 2013, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Open Source Robotics Foundation, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros/common_msgs/blob/275b09a/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

// TODO(louise) Move to `ign-msgs` and write tests
// https://github.com/osrf/lrauv/issues/151

#ifndef SENSOR_MSGS__POINT_CLOUD2_ITERATOR_HPP_
#define SENSOR_MSGS__POINT_CLOUD2_ITERATOR_HPP_

#include <ignition/msgs/pointcloud_packed.pb.h>
#include <cstdarg>
#include <string>
#include <vector>
#include <sstream>

/**
 * \brief Tools for manipulating sensor_msgs
 *
 * This file provides two sets of utilities to modify and parse PointCloudPacked
 * The first set allows you to conveniently set the fields by hand:
 * \verbatim
 *   #include <sensor_msgs/point_cloud_iterator.h>
 *   // Create a PointCloudPacked
 *   ignition::msgs::PointCloudPacked cloud_msg;
 *   // Fill some internals of the PoinCloud2 like the header/width/height ...
 *   cloud_msgs.height = 1;  cloud_msgs.width = 4;
 *   // Set the point fields to xyzrgb and resize the vector with the following command
 *   // 4 is for the number of added fields. Each come in triplet: the name of the PointCloudPacked::Field,
 *   // the number of occurrences of the type in the PointCloudPacked::Field, the type of the PointCloudPacked::Field
 *   ignition::msgs::PointCloudPackedModifier modifier(cloud_msg);
 *   modifier.setPointCloudPackedFields(4, "x", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
 *                                            "y", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
 *                                            "z", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
 *                                            "rgb", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32);
 *   // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
 *   // You have to be aware that the following function does add extra padding for backward compatibility though
 *   // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
 *   // 2 is for the number of fields to add
 *   modifier.setPointCloudPackedFieldsByString(2, "xyz", "rgb");
 *   // You can then reserve / resize as usual
 *   modifier.resize(100);
 * \endverbatim
 *
 * The second set allow you to traverse your PointCloud using an iterator:
 * \verbatim
 *   // Define some raw data we'll put in the PointCloudPacked
 *   float point_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
 *   uint8_t color_data[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
 *   // Define the iterators. When doing so, you define the Field you would like to iterate upon and
 *   // the type of you would like returned: it is not necessary the type of the PointCloudPacked::Field as sometimes
 *   // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
 *   PointCloudPackedIterator<float> iter_x(cloud_msg, "x");
 *   PointCloudPackedIterator<float> iter_y(cloud_msg, "y");
 *   PointCloudPackedIterator<float> iter_z(cloud_msg, "z");
 *   // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
 *   // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
 *   // and RGBA as A,R,G,B)
 *   PointCloudPackedIterator<uint8_t> iter_r(cloud_msg, "r");
 *   PointCloudPackedIterator<uint8_t> iter_g(cloud_msg, "g");
 *   PointCloudPackedIterator<uint8_t> iter_b(cloud_msg, "b");
 *   // Fill the PointCloudPacked
 *   for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
 *     *iter_x = point_data[3*i+0];
 *     *iter_y = point_data[3*i+1];
 *     *iter_z = point_data[3*i+2];
 *     *iter_r = color_data[3*i+0];
 *     *iter_g = color_data[3*i+1];
 *     *iter_b = color_data[3*i+2];
 *   }
 * \endverbatim
 */

namespace tethys
{
/**
 * @brief Enables modifying a ignition::msgs::PointCloudPacked like a container
 */
//       class PointCloudPackedModifier
//       {
//       public:
//         /**
//          * @brief Default constructor
//          * @param cloud_msg The ignition::msgs::PointCloudPacked to modify
//          */
//         explicit PointCloudPackedModifier(ignition::msgs::PointCloudPacked & cloud_msg);
//
//         /**
//          * @return the number of T's in the original ignition::msgs::PointCloudPacked
//          */
//         size_t size() const;
//
//         /**
//          * @param size The number of T's to reserve in the original ignition::msgs::PointCloudPacked for
//          */
//         void reserve(size_t size);
//
//         /**
//          * @param size The number of T's to change the size of the original ignition::msgs::PointCloudPacked by
//          */
//         void resize(size_t size);
//
//         /**
//          * @brief remove all T's from the original ignition::msgs::PointCloudPacked
//          */
//         void clear();
//
//         /**
//          * @brief Function setting some fields in a PointCloud and adjusting the
//          *        internals of the PointCloudPacked
//          * @param n_fields the number of fields to add. The fields are given as
//          *        triplets: name of the field as char*, number of elements in the
//          *        field, the datatype of the elements in the field
//          *
//          * E.g, you create your PointCloudPacked message with XYZ/RGB as follows:
//          * \verbatim
//          *   setPointCloudPackedFields(cloud_msg, 4, "x", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
//          *                                              "y", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
//          *                                              "z", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
//          *                                              "rgb", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32);
//          * \endverbatim
//          * WARNING: THIS DOES NOT TAKE INTO ACCOUNT ANY PADDING AS DONE UNTIL HYDRO
//          * For simple usual cases, the overloaded setPointCloudPackedFieldsByString is what you want.
//          */
//         void setPointCloudPackedFields(int n_fields, ...);
//
//         /**
//          * @brief Function setting some fields in a PointCloud and adjusting the
//          *        internals of the PointCloudPacked
//          * @param n_fields the number of fields to add. The fields are given as
//          *        strings: "xyz" (3 floats), "rgb" (3 uchar stacked in a float),
//          *        "rgba" (4 uchar stacked in a float)
//          * @return void
//          *
//          * WARNING: THIS FUNCTION DOES ADD ANY NECESSARY PADDING TRANSPARENTLY
//          */
//         void setPointCloudPackedFieldsByString(int n_fields, ...);
//
//       protected:
//         /** A reference to the original ignition::msgs::PointCloudPacked that we read */
//         ignition::msgs::PointCloudPacked & cloud_msg_;
//       };

/** Private base class for PointCloudPackedIterator and PointCloudPackedConstIterator
 * T is the type of the value on which the child class will be templated
 * TT is the type of the value to be retrieved (same as T except for constness)
 * U is the type of the raw data in PointCloudPacked (only uchar and const uchar are supported)
 * C is the type of the pointcloud to intialize from (const or not)
 * V is the derived class (yop, curiously recurring template pattern)
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
class PointCloudPackedIteratorBase
{
public:
  /**
   */
  PointCloudPackedIteratorBase();

  /**
   * @param cloud_msg The PointCloudPacked to iterate upon
   * @param field_name The field to iterate upon
   */
  PointCloudPackedIteratorBase(C & cloud_msg, const std::string & field_name);

  /** Assignment operator
   * @param iter the iterator to copy data from
   * @return a reference to *this
   */
  V<T> & operator=(const V<T> & iter);

  /** Access the i th element starting at the current pointer (useful when a field has several elements of the same
   * type)
   * @param i
   * @return a reference to the i^th value from the current position
   */
  TT & operator[](size_t i) const;

  /** Dereference the iterator. Equivalent to accessing it through [0]
   * @return the value to which the iterator is pointing
   */
  TT & operator*() const;

  /** Increase the iterator to the next element
   * @return a reference to the updated iterator
   */
  V<T> & operator++();

  /** Basic pointer addition
   * @param i the amount to increase the iterator by
   * @return an iterator with an increased position
   */
  V<T> operator+(int i);

  /** Increase the iterator by a certain amount
   * @return a reference to the updated iterator
   */
  V<T> & operator+=(int i);

  /** Compare to another iterator
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator!=(const V<T> & iter) const;

  /** Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++)
   */
  V<T> end() const;

private:
  /** Common code to set the field of the PointCloudPacked
   * @param cloud_msg the PointCloudPacked to modify
   * @param field_name the name of the field to iterate upon
   * @return the offset at which the field is found
   */
  int set_field(const ignition::msgs::PointCloudPacked & cloud_msg, const std::string & field_name);

  /** The "point_step" of the original cloud */
  int point_step_;
  /** The raw data  in uchar* where the iterator is */
  U * data_char_;
  /** The cast data where the iterator is */
  TT * data_;
  /** The end() pointer of the iterator */
  TT * data_end_;
  /** Whether the fields are stored as bigendian */
  bool is_bigendian_;
};

/**
 * \brief Class that can iterate over a PointCloudPacked
 *
 * T type of the element being iterated upon
 * E.g, you create your PointClou2 message as follows:
 * \verbatim
 *   setPointCloudPackedFieldsByString(cloud_msg, 2, "xyz", "rgb");
 * \endverbatim
 *
 * For iterating over XYZ, you do :
 * \verbatim
 *   ignition::msgs::PointCloudPackedIterator<float> iter_x(cloud_msg, "x");
 * \endverbatim
 * and then access X through iter_x[0] or *iter_x
 * You could create an iterator for Y and Z too but as they are consecutive,
 * you can just use iter_x[1] and iter_x[2]
 *
 * For iterating over RGB, you do:
 * \verbatim
 * ignition::msgs::PointCloudPackedIterator<uint8_t> iter_rgb(cloud_msg, "rgb");
 * \endverbatim
 * and then access R,G,B through  iter_rgb[0], iter_rgb[1], iter_rgb[2]
 */
template<typename T>
class PointCloudPackedIterator
  : public PointCloudPackedIteratorBase<
    T, T, char, ignition::msgs::PointCloudPacked, PointCloudPackedIterator>
{
public:
  PointCloudPackedIterator(
    ignition::msgs::PointCloudPacked & cloud_msg,
    const std::string & field_name)
  : PointCloudPackedIteratorBase<
      T, T, char,
      ignition::msgs::PointCloudPacked,
      PointCloudPackedIterator
  >::PointCloudPackedIteratorBase(cloud_msg, field_name) {}
};

/**
 * \brief Same as a PointCloudPackedIterator but for const data
 */
template<typename T>
class PointCloudPackedConstIterator
  : public PointCloudPackedIteratorBase<
    T, const T, const unsigned char, const ignition::msgs::PointCloudPacked,
    PointCloudPackedConstIterator>
{
public:
  PointCloudPackedConstIterator(
    const ignition::msgs::PointCloudPacked & cloud_msg,
    const std::string & field_name)
  : PointCloudPackedIteratorBase<
      T, const T, const unsigned char,
      const ignition::msgs::PointCloudPacked,
      PointCloudPackedConstIterator
  >::PointCloudPackedIteratorBase(cloud_msg, field_name) {}
};



/**
 * \brief Private implementation used by PointCloudPackedIterator
 * \author Vincent Rabaud
 */

/** Return the size of a datatype (which is an enum of ignition::msgs::PointCloudPacked::Field::) in bytes
 * @param datatype one of the enums of ignition::msgs::PointCloudPacked::Field::
 */
inline int sizeOfPointField(int datatype)
{
  if ((datatype == ignition::msgs::PointCloudPacked::Field::INT8) ||
    (datatype == ignition::msgs::PointCloudPacked::Field::UINT8))
  {
    return 1;
  } else if ((datatype == ignition::msgs::PointCloudPacked::Field::INT16) ||  // NOLINT
    (datatype == ignition::msgs::PointCloudPacked::Field::UINT16))
  {
    return 2;
  } else if ((datatype == ignition::msgs::PointCloudPacked::Field::INT32) ||  // NOLINT
    (datatype == ignition::msgs::PointCloudPacked::Field::UINT32) ||
    (datatype == ignition::msgs::PointCloudPacked::Field::FLOAT32))
  {
    return 4;
  } else if (datatype == ignition::msgs::PointCloudPacked::Field::FLOAT64) {
    return 8;
  } else {
    std::stringstream err;
    err << "PointCloudPacked::Field of type " << datatype << " does not exist";
    throw std::runtime_error(err.str());
  }
  return -1;
}

/** Private function that adds a PointCloudPacked::Field to the "fields" member of a PointCloudPacked
 * @param cloud_msg the PointCloudPacked to add a field to
 * @param name the name of the field
 * @param count the number of elements in the PointCloudPacked::Field
 * @param datatype the datatype of the elements
 * @param offset the offset of that element
 * @return the offset of the next PointCloudPacked::Field that will be added to the PointCloudPacked
 */
inline int addPointField(
  ignition::msgs::PointCloudPacked & cloud_msg,
  const std::string & name, int count, ignition::msgs::PointCloudPacked::Field::DataType datatype,
  int offset)
{
  auto point_field = cloud_msg.add_field();
  point_field->set_name(name);
  point_field->set_count(count);
  point_field->set_datatype(datatype);
  point_field->set_offset(offset);

  // Update the offset
  return offset + cloud_msg.field().size() * sizeOfPointField(datatype);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//             inline PointCloudPackedModifier::PointCloudPackedModifier(
//               ignition::msgs::PointCloudPacked & cloud_msg)
//             : cloud_msg_(cloud_msg)
//             {
//             }
//
//             inline size_t PointCloudPackedModifier::size() const
//             {
//               return cloud_msg_.data().size() / cloud_msg_.point_step();
//             }
//
//             inline void PointCloudPackedModifier::reserve(size_t size)
//             {
//               cloud_msg_.mutable_data->reserve(size * cloud_msg_.point_step());
//             }
//
//             inline void PointCloudPackedModifier::resize(size_t size)
//             {
//               cloud_msg_.mutable_data()->resize(size * cloud_msg_.point_step());
//
//               // Update height/width
//               if (cloud_msg_.height() == 1) {
//                 cloud_msg_.width() = static_cast<uint32_t>(size);
//                 cloud_msg_.row_step() = static_cast<uint32_t>(size * cloud_msg_.point_step());
//               } else {
//                 if (cloud_msg_.width() == 1) {
//                   cloud_msg_.height() = static_cast<uint32_t>(size);
//                 } else {
//                   cloud_msg_.height() = 1;
//                   cloud_msg_.width() = static_cast<uint32_t>(size);
//                   cloud_msg_.row_step() = static_cast<uint32_t>(size * cloud_msg_.point_step());
//                 }
//               }
//             }
//
//             inline void PointCloudPackedModifier::clear()
//             {
//               cloud_msg_.mutable_data()->clear();
//
//               // Update height/width
//               if (cloud_msg_.height() == 1) {
//                 cloud_msg_.row_step() = cloud_msg_.width() = 0;
//               } else {
//                 if (cloud_msg_.width() == 1) {
//                   cloud_msg_.height() = 0;
//                 } else {
//                   cloud_msg_.row_step() = cloud_msg_.width() = cloud_msg_.height() = 0;
//                 }
//               }
//             }
//
//
//             /**
//              * @brief Function setting some fields in a PointCloud and adjusting the
//              *        internals of the PointCloudPacked
//              * @param n_fields the number of fields to add. The fields are given as
//              *        triplets: name of the field as char*, number of elements in the
//              *        field, the datatype of the elements in the field
//              *
//              * E.g, you create your PointCloudPacked message with XYZ/RGB as follows:
//              * <PRE>
//              *   setPointCloudPackedFieldsByString(cloud_msg, 4, "x", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
//              *                                              "y", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
//              *                                              "z", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
//              *                                              "rgb", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32);
//              * </PRE>
//              * WARNING: THIS DOES NOT TAKE INTO ACCOUNT ANY PADDING AS DONE UNTIL HYDRO
//              * For simple usual cases, the overloaded setPointCloudPackedFieldsByString is what you want.
//              */
//             inline void PointCloudPackedModifier::setPointCloudPackedFields(int n_fields, ...)
//             {
//               cloud_msg_.fields.clear();
//               cloud_msg_.fields.reserve(n_fields);
//               va_list vl;
//               va_start(vl, n_fields);
//               int offset = 0;
//               for (int i = 0; i < n_fields; ++i) {
//                 // Create the corresponding PointCloudPacked::Field
//                 std::string name(va_arg(vl, char *));
//                 int count(va_arg(vl, int));
//                 int datatype(va_arg(vl, int));
//                 offset = addPointField(cloud_msg_, name, count, datatype, offset);
//               }
//               va_end(vl);
//
//               // Resize the point cloud accordingly
//               cloud_msg_.point_step = offset;
//               cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step();
//               cloud_msg_.data.resize(cloud_msg_.height * cloud_msg_.row_step);
//             }
//
//             /**
//              * @brief Function setting some fields in a PointCloud and adjusting the
//              *        internals of the PointCloudPacked
//              * @param n_fields the number of fields to add. The fields are given as
//              *        strings: "xyz" (3 floats), "rgb" (3 uchar stacked in a float),
//              *        "rgba" (4 uchar stacked in a float)
//              * @return void
//              *
//              * WARNING: THIS FUNCTION DOES ADD ANY NECESSARY PADDING TRANSPARENTLY
//              */
//             inline void PointCloudPackedModifier::setPointCloudPackedFieldsByString(int n_fields, ...)
//             {
//               cloud_msg_.fields.clear();
//               cloud_msg_.fields.reserve(n_fields);
//               va_list vl;
//               va_start(vl, n_fields);
//               int offset = 0;
//               for (int i = 0; i < n_fields; ++i) {
//                 // Create the corresponding PointCloudPacked::Fields
//                 std::string
//                   field_name = std::string(va_arg(vl, char *));
//                 if (field_name == "xyz") {
//                   ignition::msgs::PointCloudPacked::Field point_field;
//                   // Do x, y and z
//                   offset = addPointField(
//                     cloud_msg_, "x", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32, offset);
//                   offset = addPointField(
//                     cloud_msg_, "y", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32, offset);
//                   offset = addPointField(
//                     cloud_msg_, "z", 1, ignition::msgs::PointCloudPacked::Field::FLOAT32, offset);
//                   offset += sizeOfPointField(ignition::msgs::PointCloudPacked::Field::FLOAT32);
//                 } else {
//                   if ((field_name == "rgb") || (field_name == "rgba")) {
//                     offset = addPointField(
//                       cloud_msg_, field_name, 1, ignition::msgs::PointCloudPacked::Field::FLOAT32,
//                       offset);
//                     offset += 3 * sizeOfPointField(ignition::msgs::PointCloudPacked::Field::FLOAT32);
//                   } else {
//                     va_end(vl);
//                     throw std::runtime_error("Field " + field_name + " does not exist");
//                   }
//                 }
//               }
//               va_end(vl);
//
//               // Resize the point cloud accordingly
//               cloud_msg_.point_step = offset;
//               cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step();
//               cloud_msg_.data.resize(cloud_msg_.height * cloud_msg_.row_step);
//             }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
PointCloudPackedIteratorBase<T, TT, U, C, V>::PointCloudPackedIteratorBase()
: data_char_(0), data_(0), data_end_(0)
{
}

/**
 * @param cloud_msg The PointCloudPacked to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
PointCloudPackedIteratorBase<T, TT, U, C, V>::PointCloudPackedIteratorBase(
  C & cloud_msg, const std::string & field_name)
{
  int offset = set_field(cloud_msg, field_name);

  data_char_ = const_cast<char *>(&(cloud_msg.data().front())) + offset;
  data_ = reinterpret_cast<TT *>(data_char_);
  data_end_ = reinterpret_cast<TT *>(const_cast<char *>(&(cloud_msg.data().back())) + 1 + offset);
}

/** Assignment operator
 * @param iter the iterator to copy data from
 * @return a reference to *this
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
V<T> & PointCloudPackedIteratorBase<T, TT, U, C, V>::operator=(const V<T> & iter)
{
  if (this != &iter) {
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
template<typename T, typename TT, typename U, typename C, template<typename> class V>
TT & PointCloudPackedIteratorBase<T, TT, U, C, V>::operator[](size_t i) const
{
  return *(data_ + i);
}

/** Dereference the iterator. Equivalent to accessing it through [0]
 * @return the value to which the iterator is pointing
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
TT & PointCloudPackedIteratorBase<T, TT, U, C, V>::operator*() const
{
  return *data_;
}

/** Increase the iterator to the next element
 * @return a reference to the updated iterator
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
V<T> & PointCloudPackedIteratorBase<T, TT, U, C, V>::operator++()
{
  data_char_ += point_step_;
  data_ = reinterpret_cast<TT *>(data_char_);
  return *static_cast<V<T> *>(this);
}

/** Basic pointer addition
 * @param i the amount to increase the iterator by
 * @return an iterator with an increased position
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
V<T> PointCloudPackedIteratorBase<T, TT, U, C, V>::operator+(int i)
{
  V<T> res = *static_cast<V<T> *>(this);

  res.data_char_ += i * point_step_;
  res.data_ = reinterpret_cast<TT *>(res.data_char_);

  return res;
}

/** Increase the iterator by a certain amount
 * @return a reference to the updated iterator
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
V<T> & PointCloudPackedIteratorBase<T, TT, U, C, V>::operator+=(int i)
{
  data_char_ += i * point_step_;
  data_ = reinterpret_cast<TT *>(data_char_);
  return *static_cast<V<T> *>(this);
}

/** Compare to another iterator
 * @return whether the current iterator points to a different address than the other one
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
bool PointCloudPackedIteratorBase<T, TT, U, C, V>::operator!=(const V<T> & iter) const
{
  return iter.data_ != data_;
}

/** Return the end iterator
 * @return the end iterator (useful when performing normal iterator processing with ++)
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
V<T> PointCloudPackedIteratorBase<T, TT, U, C, V>::end() const
{
  V<T> res = *static_cast<const V<T> *>(this);
  res.data_ = data_end_;
  return res;
}

/** Common code to set the field of the PointCloudPacked
  * @param cloud_msg the PointCloudPacked to modify
  * @param field_name the name of the field to iterate upon
  * @return the offset at which the field is found
  */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
int PointCloudPackedIteratorBase<T, TT, U, C, V>::set_field(
  const ignition::msgs::PointCloudPacked & cloud_msg, const std::string & field_name)
{
  is_bigendian_ = cloud_msg.is_bigendian();
  point_step_ = cloud_msg.point_step();
  // make sure the channel is valid
  auto field_iter = cloud_msg.field().begin();
  auto field_end = cloud_msg.field().end();
  while ((field_iter != field_end) && (field_iter->name() != field_name)) {
    ++field_iter;
  }

  if (field_iter == field_end) {
    // Handle the special case of r,g,b,a (we assume they are understood as the
    // channels of an rgb or rgba field)
    if ((field_name == "r") || (field_name == "g") || (field_name == "b") || (field_name == "a")) {
      // Check that rgb or rgba is present
      field_iter = cloud_msg.field().begin();
      while ((field_iter != field_end) && (field_iter->name() != "rgb") &&
        (field_iter->name() != "rgba"))
      {
        ++field_iter;
      }
      if (field_iter == field_end) {
        throw std::runtime_error("Field " + field_name + " does not exist");
      }
      if (field_name == "r") {
        if (is_bigendian_) {
          return field_iter->offset() + 1;
        } else {
          return field_iter->offset() + 2;
        }
      }
      if (field_name == "g") {
        if (is_bigendian_) {
          return field_iter->offset() + 2;
        } else {
          return field_iter->offset() + 1;
        }
      }
      if (field_name == "b") {
        if (is_bigendian_) {
          return field_iter->offset() + 3;
        } else {
          return field_iter->offset() + 0;
        }
      }
      if (field_name == "a") {
        if (is_bigendian_) {
          return field_iter->offset() + 0;
        } else {
          return field_iter->offset() + 3;
        }
      }
    } else {
      throw std::runtime_error("Field " + field_name + " does not exist");
    }
  }

  return field_iter->offset();
}

}

#endif  // SENSOR_MSGS__IMPL__POINT_CLOUD2_ITERATOR_HPP_

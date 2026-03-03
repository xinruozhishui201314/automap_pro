// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automap_pro:action/RunHBA.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__STRUCT_HPP_
#define AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_Goal __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_Goal __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_Goal_
{
  using Type = RunHBA_Goal_<ContainerAllocator>;

  explicit RunHBA_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_path = "";
      this->total_layer_num = 0l;
      this->thread_num = 0l;
      this->enable_gps = false;
    }
  }

  explicit RunHBA_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_path = "";
      this->total_layer_num = 0l;
      this->thread_num = 0l;
      this->enable_gps = false;
    }
  }

  // field types and members
  using _data_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_path_type data_path;
  using _total_layer_num_type =
    int32_t;
  _total_layer_num_type total_layer_num;
  using _thread_num_type =
    int32_t;
  _thread_num_type thread_num;
  using _enable_gps_type =
    bool;
  _enable_gps_type enable_gps;

  // setters for named parameter idiom
  Type & set__data_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data_path = _arg;
    return *this;
  }
  Type & set__total_layer_num(
    const int32_t & _arg)
  {
    this->total_layer_num = _arg;
    return *this;
  }
  Type & set__thread_num(
    const int32_t & _arg)
  {
    this->thread_num = _arg;
    return *this;
  }
  Type & set__enable_gps(
    const bool & _arg)
  {
    this->enable_gps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_Goal
    std::shared_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_Goal
    std::shared_ptr<automap_pro::action::RunHBA_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_Goal_ & other) const
  {
    if (this->data_path != other.data_path) {
      return false;
    }
    if (this->total_layer_num != other.total_layer_num) {
      return false;
    }
    if (this->thread_num != other.thread_num) {
      return false;
    }
    if (this->enable_gps != other.enable_gps) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_Goal_

// alias to use template instance with default allocator
using RunHBA_Goal =
  automap_pro::action::RunHBA_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_Result __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_Result __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_Result_
{
  using Type = RunHBA_Result_<ContainerAllocator>;

  explicit RunHBA_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->output_path = "";
      this->elapsed_seconds = 0.0;
      this->final_mme = 0.0;
      this->total_keyframes = 0l;
    }
  }

  explicit RunHBA_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : output_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->output_path = "";
      this->elapsed_seconds = 0.0;
      this->final_mme = 0.0;
      this->total_keyframes = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _output_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _output_path_type output_path;
  using _elapsed_seconds_type =
    double;
  _elapsed_seconds_type elapsed_seconds;
  using _final_mme_type =
    double;
  _final_mme_type final_mme;
  using _total_keyframes_type =
    int32_t;
  _total_keyframes_type total_keyframes;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__output_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->output_path = _arg;
    return *this;
  }
  Type & set__elapsed_seconds(
    const double & _arg)
  {
    this->elapsed_seconds = _arg;
    return *this;
  }
  Type & set__final_mme(
    const double & _arg)
  {
    this->final_mme = _arg;
    return *this;
  }
  Type & set__total_keyframes(
    const int32_t & _arg)
  {
    this->total_keyframes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_Result
    std::shared_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_Result
    std::shared_ptr<automap_pro::action::RunHBA_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->output_path != other.output_path) {
      return false;
    }
    if (this->elapsed_seconds != other.elapsed_seconds) {
      return false;
    }
    if (this->final_mme != other.final_mme) {
      return false;
    }
    if (this->total_keyframes != other.total_keyframes) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_Result_

// alias to use template instance with default allocator
using RunHBA_Result =
  automap_pro::action::RunHBA_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro


#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_Feedback __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_Feedback_
{
  using Type = RunHBA_Feedback_<ContainerAllocator>;

  explicit RunHBA_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_layer = 0l;
      this->total_layers = 0l;
      this->progress_percent = 0.0f;
      this->layer_elapsed_ms = 0.0f;
    }
  }

  explicit RunHBA_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_layer = 0l;
      this->total_layers = 0l;
      this->progress_percent = 0.0f;
      this->layer_elapsed_ms = 0.0f;
    }
  }

  // field types and members
  using _current_layer_type =
    int32_t;
  _current_layer_type current_layer;
  using _total_layers_type =
    int32_t;
  _total_layers_type total_layers;
  using _progress_percent_type =
    float;
  _progress_percent_type progress_percent;
  using _layer_elapsed_ms_type =
    float;
  _layer_elapsed_ms_type layer_elapsed_ms;

  // setters for named parameter idiom
  Type & set__current_layer(
    const int32_t & _arg)
  {
    this->current_layer = _arg;
    return *this;
  }
  Type & set__total_layers(
    const int32_t & _arg)
  {
    this->total_layers = _arg;
    return *this;
  }
  Type & set__progress_percent(
    const float & _arg)
  {
    this->progress_percent = _arg;
    return *this;
  }
  Type & set__layer_elapsed_ms(
    const float & _arg)
  {
    this->layer_elapsed_ms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_Feedback
    std::shared_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_Feedback
    std::shared_ptr<automap_pro::action::RunHBA_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_Feedback_ & other) const
  {
    if (this->current_layer != other.current_layer) {
      return false;
    }
    if (this->total_layers != other.total_layers) {
      return false;
    }
    if (this->progress_percent != other.progress_percent) {
      return false;
    }
    if (this->layer_elapsed_ms != other.layer_elapsed_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_Feedback_

// alias to use template instance with default allocator
using RunHBA_Feedback =
  automap_pro::action::RunHBA_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "automap_pro/action/detail/run_hba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_SendGoal_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_SendGoal_Request_
{
  using Type = RunHBA_SendGoal_Request_<ContainerAllocator>;

  explicit RunHBA_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit RunHBA_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    automap_pro::action::RunHBA_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const automap_pro::action::RunHBA_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_SendGoal_Request
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_SendGoal_Request
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_SendGoal_Request_

// alias to use template instance with default allocator
using RunHBA_SendGoal_Request =
  automap_pro::action::RunHBA_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_SendGoal_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_SendGoal_Response_
{
  using Type = RunHBA_SendGoal_Response_<ContainerAllocator>;

  explicit RunHBA_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit RunHBA_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_SendGoal_Response
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_SendGoal_Response
    std::shared_ptr<automap_pro::action::RunHBA_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_SendGoal_Response_

// alias to use template instance with default allocator
using RunHBA_SendGoal_Response =
  automap_pro::action::RunHBA_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro

namespace automap_pro
{

namespace action
{

struct RunHBA_SendGoal
{
  using Request = automap_pro::action::RunHBA_SendGoal_Request;
  using Response = automap_pro::action::RunHBA_SendGoal_Response;
};

}  // namespace action

}  // namespace automap_pro


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_GetResult_Request __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_GetResult_Request_
{
  using Type = RunHBA_GetResult_Request_<ContainerAllocator>;

  explicit RunHBA_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit RunHBA_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_GetResult_Request
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_GetResult_Request
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_GetResult_Request_

// alias to use template instance with default allocator
using RunHBA_GetResult_Request =
  automap_pro::action::RunHBA_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro


// Include directives for member types
// Member 'result'
// already included above
// #include "automap_pro/action/detail/run_hba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_GetResult_Response __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_GetResult_Response_
{
  using Type = RunHBA_GetResult_Response_<ContainerAllocator>;

  explicit RunHBA_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit RunHBA_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    automap_pro::action::RunHBA_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const automap_pro::action::RunHBA_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_GetResult_Response
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_GetResult_Response
    std::shared_ptr<automap_pro::action::RunHBA_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_GetResult_Response_

// alias to use template instance with default allocator
using RunHBA_GetResult_Response =
  automap_pro::action::RunHBA_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro

namespace automap_pro
{

namespace action
{

struct RunHBA_GetResult
{
  using Request = automap_pro::action::RunHBA_GetResult_Request;
  using Response = automap_pro::action::RunHBA_GetResult_Response;
};

}  // namespace action

}  // namespace automap_pro


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "automap_pro/action/detail/run_hba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automap_pro__action__RunHBA_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__automap_pro__action__RunHBA_FeedbackMessage __declspec(deprecated)
#endif

namespace automap_pro
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RunHBA_FeedbackMessage_
{
  using Type = RunHBA_FeedbackMessage_<ContainerAllocator>;

  explicit RunHBA_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit RunHBA_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    automap_pro::action::RunHBA_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const automap_pro::action::RunHBA_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automap_pro__action__RunHBA_FeedbackMessage
    std::shared_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automap_pro__action__RunHBA_FeedbackMessage
    std::shared_ptr<automap_pro::action::RunHBA_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunHBA_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunHBA_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunHBA_FeedbackMessage_

// alias to use template instance with default allocator
using RunHBA_FeedbackMessage =
  automap_pro::action::RunHBA_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace automap_pro

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace automap_pro
{

namespace action
{

struct RunHBA
{
  /// The goal message defined in the action definition.
  using Goal = automap_pro::action::RunHBA_Goal;
  /// The result message defined in the action definition.
  using Result = automap_pro::action::RunHBA_Result;
  /// The feedback message defined in the action definition.
  using Feedback = automap_pro::action::RunHBA_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = automap_pro::action::RunHBA_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = automap_pro::action::RunHBA_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = automap_pro::action::RunHBA_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct RunHBA RunHBA;

}  // namespace action

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__STRUCT_HPP_

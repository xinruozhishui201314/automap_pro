// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automap_pro:action/RunHBA.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__BUILDER_HPP_
#define AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automap_pro/action/detail/run_hba__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_Goal_enable_gps
{
public:
  explicit Init_RunHBA_Goal_enable_gps(::automap_pro::action::RunHBA_Goal & msg)
  : msg_(msg)
  {}
  ::automap_pro::action::RunHBA_Goal enable_gps(::automap_pro::action::RunHBA_Goal::_enable_gps_type arg)
  {
    msg_.enable_gps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Goal msg_;
};

class Init_RunHBA_Goal_thread_num
{
public:
  explicit Init_RunHBA_Goal_thread_num(::automap_pro::action::RunHBA_Goal & msg)
  : msg_(msg)
  {}
  Init_RunHBA_Goal_enable_gps thread_num(::automap_pro::action::RunHBA_Goal::_thread_num_type arg)
  {
    msg_.thread_num = std::move(arg);
    return Init_RunHBA_Goal_enable_gps(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Goal msg_;
};

class Init_RunHBA_Goal_total_layer_num
{
public:
  explicit Init_RunHBA_Goal_total_layer_num(::automap_pro::action::RunHBA_Goal & msg)
  : msg_(msg)
  {}
  Init_RunHBA_Goal_thread_num total_layer_num(::automap_pro::action::RunHBA_Goal::_total_layer_num_type arg)
  {
    msg_.total_layer_num = std::move(arg);
    return Init_RunHBA_Goal_thread_num(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Goal msg_;
};

class Init_RunHBA_Goal_data_path
{
public:
  Init_RunHBA_Goal_data_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunHBA_Goal_total_layer_num data_path(::automap_pro::action::RunHBA_Goal::_data_path_type arg)
  {
    msg_.data_path = std::move(arg);
    return Init_RunHBA_Goal_total_layer_num(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_Goal>()
{
  return automap_pro::action::builder::Init_RunHBA_Goal_data_path();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_Result_total_keyframes
{
public:
  explicit Init_RunHBA_Result_total_keyframes(::automap_pro::action::RunHBA_Result & msg)
  : msg_(msg)
  {}
  ::automap_pro::action::RunHBA_Result total_keyframes(::automap_pro::action::RunHBA_Result::_total_keyframes_type arg)
  {
    msg_.total_keyframes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Result msg_;
};

class Init_RunHBA_Result_final_mme
{
public:
  explicit Init_RunHBA_Result_final_mme(::automap_pro::action::RunHBA_Result & msg)
  : msg_(msg)
  {}
  Init_RunHBA_Result_total_keyframes final_mme(::automap_pro::action::RunHBA_Result::_final_mme_type arg)
  {
    msg_.final_mme = std::move(arg);
    return Init_RunHBA_Result_total_keyframes(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Result msg_;
};

class Init_RunHBA_Result_elapsed_seconds
{
public:
  explicit Init_RunHBA_Result_elapsed_seconds(::automap_pro::action::RunHBA_Result & msg)
  : msg_(msg)
  {}
  Init_RunHBA_Result_final_mme elapsed_seconds(::automap_pro::action::RunHBA_Result::_elapsed_seconds_type arg)
  {
    msg_.elapsed_seconds = std::move(arg);
    return Init_RunHBA_Result_final_mme(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Result msg_;
};

class Init_RunHBA_Result_output_path
{
public:
  explicit Init_RunHBA_Result_output_path(::automap_pro::action::RunHBA_Result & msg)
  : msg_(msg)
  {}
  Init_RunHBA_Result_elapsed_seconds output_path(::automap_pro::action::RunHBA_Result::_output_path_type arg)
  {
    msg_.output_path = std::move(arg);
    return Init_RunHBA_Result_elapsed_seconds(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Result msg_;
};

class Init_RunHBA_Result_success
{
public:
  Init_RunHBA_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunHBA_Result_output_path success(::automap_pro::action::RunHBA_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RunHBA_Result_output_path(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_Result>()
{
  return automap_pro::action::builder::Init_RunHBA_Result_success();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_Feedback_layer_elapsed_ms
{
public:
  explicit Init_RunHBA_Feedback_layer_elapsed_ms(::automap_pro::action::RunHBA_Feedback & msg)
  : msg_(msg)
  {}
  ::automap_pro::action::RunHBA_Feedback layer_elapsed_ms(::automap_pro::action::RunHBA_Feedback::_layer_elapsed_ms_type arg)
  {
    msg_.layer_elapsed_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Feedback msg_;
};

class Init_RunHBA_Feedback_progress_percent
{
public:
  explicit Init_RunHBA_Feedback_progress_percent(::automap_pro::action::RunHBA_Feedback & msg)
  : msg_(msg)
  {}
  Init_RunHBA_Feedback_layer_elapsed_ms progress_percent(::automap_pro::action::RunHBA_Feedback::_progress_percent_type arg)
  {
    msg_.progress_percent = std::move(arg);
    return Init_RunHBA_Feedback_layer_elapsed_ms(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Feedback msg_;
};

class Init_RunHBA_Feedback_total_layers
{
public:
  explicit Init_RunHBA_Feedback_total_layers(::automap_pro::action::RunHBA_Feedback & msg)
  : msg_(msg)
  {}
  Init_RunHBA_Feedback_progress_percent total_layers(::automap_pro::action::RunHBA_Feedback::_total_layers_type arg)
  {
    msg_.total_layers = std::move(arg);
    return Init_RunHBA_Feedback_progress_percent(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Feedback msg_;
};

class Init_RunHBA_Feedback_current_layer
{
public:
  Init_RunHBA_Feedback_current_layer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunHBA_Feedback_total_layers current_layer(::automap_pro::action::RunHBA_Feedback::_current_layer_type arg)
  {
    msg_.current_layer = std::move(arg);
    return Init_RunHBA_Feedback_total_layers(msg_);
  }

private:
  ::automap_pro::action::RunHBA_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_Feedback>()
{
  return automap_pro::action::builder::Init_RunHBA_Feedback_current_layer();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_SendGoal_Request_goal
{
public:
  explicit Init_RunHBA_SendGoal_Request_goal(::automap_pro::action::RunHBA_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::automap_pro::action::RunHBA_SendGoal_Request goal(::automap_pro::action::RunHBA_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_SendGoal_Request msg_;
};

class Init_RunHBA_SendGoal_Request_goal_id
{
public:
  Init_RunHBA_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunHBA_SendGoal_Request_goal goal_id(::automap_pro::action::RunHBA_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RunHBA_SendGoal_Request_goal(msg_);
  }

private:
  ::automap_pro::action::RunHBA_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_SendGoal_Request>()
{
  return automap_pro::action::builder::Init_RunHBA_SendGoal_Request_goal_id();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_SendGoal_Response_stamp
{
public:
  explicit Init_RunHBA_SendGoal_Response_stamp(::automap_pro::action::RunHBA_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::action::RunHBA_SendGoal_Response stamp(::automap_pro::action::RunHBA_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_SendGoal_Response msg_;
};

class Init_RunHBA_SendGoal_Response_accepted
{
public:
  Init_RunHBA_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunHBA_SendGoal_Response_stamp accepted(::automap_pro::action::RunHBA_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_RunHBA_SendGoal_Response_stamp(msg_);
  }

private:
  ::automap_pro::action::RunHBA_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_SendGoal_Response>()
{
  return automap_pro::action::builder::Init_RunHBA_SendGoal_Response_accepted();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_GetResult_Request_goal_id
{
public:
  Init_RunHBA_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::automap_pro::action::RunHBA_GetResult_Request goal_id(::automap_pro::action::RunHBA_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_GetResult_Request>()
{
  return automap_pro::action::builder::Init_RunHBA_GetResult_Request_goal_id();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_GetResult_Response_result
{
public:
  explicit Init_RunHBA_GetResult_Response_result(::automap_pro::action::RunHBA_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::automap_pro::action::RunHBA_GetResult_Response result(::automap_pro::action::RunHBA_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_GetResult_Response msg_;
};

class Init_RunHBA_GetResult_Response_status
{
public:
  Init_RunHBA_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunHBA_GetResult_Response_result status(::automap_pro::action::RunHBA_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RunHBA_GetResult_Response_result(msg_);
  }

private:
  ::automap_pro::action::RunHBA_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_GetResult_Response>()
{
  return automap_pro::action::builder::Init_RunHBA_GetResult_Response_status();
}

}  // namespace automap_pro


namespace automap_pro
{

namespace action
{

namespace builder
{

class Init_RunHBA_FeedbackMessage_feedback
{
public:
  explicit Init_RunHBA_FeedbackMessage_feedback(::automap_pro::action::RunHBA_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::automap_pro::action::RunHBA_FeedbackMessage feedback(::automap_pro::action::RunHBA_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automap_pro::action::RunHBA_FeedbackMessage msg_;
};

class Init_RunHBA_FeedbackMessage_goal_id
{
public:
  Init_RunHBA_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunHBA_FeedbackMessage_feedback goal_id(::automap_pro::action::RunHBA_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RunHBA_FeedbackMessage_feedback(msg_);
  }

private:
  ::automap_pro::action::RunHBA_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automap_pro::action::RunHBA_FeedbackMessage>()
{
  return automap_pro::action::builder::Init_RunHBA_FeedbackMessage_goal_id();
}

}  // namespace automap_pro

#endif  // AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__BUILDER_HPP_

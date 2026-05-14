#include "ui_cmd_server.h"




UiCmdServer::UiCmdServer() {
    robot_ = controller::createController();
    if (!initDds()) {
        throw std::runtime_error("DDS init failed");
    }

    running_ = true;

    state_thread_ = std::thread([this]() {
        while (running_) {

            publishRobotState();

            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz
        }
    });
    spdlog::info("DDS初始化完成");
    
}

UiCmdServer::~UiCmdServer() {
    running_ = false;
    if (state_thread_.joinable()) {
        state_thread_.join();
    }
    if (csp_thread_.joinable()) {
        csp_thread_.join();
    }

    
    shutdownDds();
    spdlog::info("DDS实体已被销毁");
}

bool UiCmdServer::initDds() {
      participant_ = dds_create_participant(200, NULL, NULL);
      if (participant_ < 0) {
          DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant_));
          return false;
      }
      spdlog::info("participant created successfully");

      subscriber_ = dds_create_subscriber(participant_, NULL, NULL);
      if (subscriber_ < 0) {
          DDS_FATAL("dds_create_subscriber: %s\n", dds_strretcode(-subscriber_));
          return false;
      }
      spdlog::info("subscriber created successfully");
      
      listener_ = dds_create_listener(this);

      //开始指令
      topic_start = dds_create_topic(
          participant_,
          &zdl_msg_dds__StartRequest_desc,
          "zdl/msg/dds/request_ui_cmd/start_request",
          NULL, NULL
      );
      if (topic_start < 0) {
          DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_start));
          return false;
      }
      spdlog::info("topic_start created successfully");

      dds_qos_t* qos_start = dds_create_qos();
      dds_qset_reliability(qos_start, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
      dds_qset_durability(qos_start, DDS_DURABILITY_VOLATILE);

      
      dds_lset_data_available(listener_, UiCmdServer::on_data_available);
      dds_lset_subscription_matched(listener_, UiCmdServer::on_subscription_matched);
      
      dds_lset_publication_matched(listener_, UiCmdServer::on_publication_matched);

      reader_start = dds_create_reader(subscriber_, topic_start, qos_start, listener_);
    
      dds_delete_qos(qos_start);
      

      if (reader_start < 0) {
          DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_start));
          return false;
      }
      spdlog::info("reader_start created successfully");

    

      //停止指令
      topic_stop = dds_create_topic(
          participant_,
          &zdl_msg_dds__StopRequest_desc,
          "zdl/msg/dds/request_ui_cmd/stop_request",
          NULL, NULL
      );
      if (topic_stop < 0) {
          DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_stop));
          return false;
      }
      spdlog::info("topic_stop created successfully");

      dds_qos_t* qos_stop = dds_create_qos();
      dds_qset_reliability(qos_stop, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
      dds_qset_durability(qos_stop, DDS_DURABILITY_VOLATILE);



      reader_stop = dds_create_reader(subscriber_, topic_stop, qos_stop, listener_);

      dds_delete_qos(qos_stop);
    

      if (reader_stop < 0) {
          DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_stop));
          return false;
      }
      spdlog::info("reader_stop created successfully");


      //使能指令
      topic_enable = dds_create_topic(
          participant_,
          &zdl_msg_dds__EnableRequest_desc,
          "zdl/msg/dds/request_ui_cmd/enable_request",
          NULL, NULL
      );
      if (topic_enable < 0) {
          DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_enable));
          return false;
      }
      spdlog::info("topic_enable created successfully");

      dds_qos_t* qos_enable = dds_create_qos();
      dds_qset_reliability(qos_enable, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
      dds_qset_durability(qos_enable, DDS_DURABILITY_VOLATILE);

      reader_enable = dds_create_reader(subscriber_, topic_enable, qos_enable, listener_);//停止指令
      dds_delete_qos(qos_enable);
    

      if (reader_enable < 0) {
          DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_enable));
          return false;
      }
      spdlog::info("reader_enable created successfully");


      

      //失能指令
      topic_disable = dds_create_topic(
          participant_,
          &zdl_msg_dds__DisableRequest_desc,
          "zdl/msg/dds/request_ui_cmd/disable_request",
          NULL, NULL
      );
      if (topic_disable < 0) {
          DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_disable));
          return false;
      }
      spdlog::info("topic_disable created successfully");

      dds_qos_t* qos_disable = dds_create_qos();
      dds_qset_reliability(qos_disable, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
      dds_qset_durability(qos_disable, DDS_DURABILITY_VOLATILE);

      reader_disable = dds_create_reader(subscriber_, topic_disable, qos_disable, listener_);
      dds_delete_qos(qos_disable);
    

      if (reader_disable < 0) {
          DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_disable));
          return false;
      }
      spdlog::info("reader_disable created successfully");



      //重置零点指令
      topic_set_zero_position = dds_create_topic(
          participant_,
          &zdl_msg_dds__SetZeroPositionRequest_desc,
          "zdl/msg/dds/request_ui_cmd/set_zero_position_request",
          NULL, NULL
      );
      if (topic_set_zero_position < 0) {
          DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_set_zero_position));
          return false;
      }
      spdlog::info("topic_set_zero_position created successfully");

      dds_qos_t* qos_set_zero_position = dds_create_qos();
      dds_qset_reliability(qos_set_zero_position, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
      dds_qset_durability(qos_set_zero_position, DDS_DURABILITY_VOLATILE);

      reader_set_zero_position = dds_create_reader(subscriber_, topic_set_zero_position, qos_set_zero_position, listener_);
      dds_delete_qos(qos_set_zero_position);
    

      if (reader_set_zero_position < 0) {
          DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_set_zero_position));
          return false;
      }
      spdlog::info("reader_set_zero_position created successfully");



      //回零指令
      topic_return_zero = dds_create_topic(
          participant_,
          &zdl_msg_dds__ReturnZeroRequest_desc,
          "zdl/msg/dds/request_ui_cmd/return_zero_request",
          NULL, NULL
      );
      if (topic_return_zero < 0) {
          DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_return_zero));
          return false;
      }
      spdlog::info("topic_return_zero created successfully");

      dds_qos_t* qos_return_zero = dds_create_qos();
      dds_qset_reliability(qos_return_zero, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
      dds_qset_durability(qos_return_zero, DDS_DURABILITY_VOLATILE);

      reader_return_zero = dds_create_reader(subscriber_, topic_return_zero, qos_return_zero, listener_);
      dds_delete_qos(qos_return_zero);
    

      if (reader_return_zero < 0) {
          DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_return_zero));
          return false;
      }
      spdlog::info("reader_return_zero created successfully");




      //点动指令
      topic_jog_command = dds_create_topic(
        participant_,
        &zdl_msg_dds__JogCommand_desc,
        "zdl/msg/dds/request_ui_cmd/jog_command",
        NULL,
        NULL
    );
    if (topic_jog_command < 0) {
          DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_jog_command));
          return false;
      }
      spdlog::info("topic_jog_command created successfully");

    dds_qos_t* qos_jog_command = dds_create_qos();
    dds_qset_reliability(qos_jog_command, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_durability(qos_jog_command, DDS_DURABILITY_VOLATILE);

    reader_jog_command = dds_create_reader(subscriber_, topic_jog_command, qos_jog_command, listener_);
    dds_delete_qos(qos_jog_command);
    if (reader_jog_command < 0) {
          DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_jog_command));
          return false;
      }
      spdlog::info("reader_jog_command created successfully");




    publisher_ = dds_create_publisher(participant_, NULL, NULL);
    if (publisher_ < 0) {
        DDS_FATAL("dds_create_publisher: %s\n", dds_strretcode(-publisher_));
        return false;
    }
    spdlog::info("publisher created successfully");






    // 腰部目标
    topic_waist_target = dds_create_topic(
        participant_,
        &zdl_msg_dds__WaistTarget_desc,
        "zdl/msg/dds/request_ui_cmd/waist_target",
        NULL, NULL
    );
    if(topic_waist_target < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_waist_target));
        return false;
    }
    spdlog::info("topic_waist_target created successfully");
    
     dds_qos_t* qos_waist_target = dds_create_qos();
    dds_qset_reliability(qos_waist_target, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_durability(qos_waist_target, DDS_DURABILITY_VOLATILE);

    reader_waist_target = dds_create_reader(subscriber_, topic_waist_target, qos_waist_target, listener_);

    dds_delete_qos(qos_waist_target);

        if (reader_waist_target < 0) {
            DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_waist_target));
            return false;
        }
        spdlog::info("reader_waist_target created successfully");



    // 轮子停止指令
    topic_wheel_stop = dds_create_topic(
        participant_,
        &zdl_msg_dds__WheelStop_desc,
        "zdl/msg/dds/request_ui_cmd/wheel_stop",
        NULL, NULL
    );
    if (topic_wheel_stop < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_wheel_stop));
        return false;
    }
    spdlog::info("topic_wheel_stop created successfully");
    
    dds_qos_t* qos_wheel_stop = dds_create_qos();
    dds_qset_reliability(qos_wheel_stop, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_durability(qos_wheel_stop, DDS_DURABILITY_VOLATILE);

    reader_wheel_stop = dds_create_reader(subscriber_, topic_wheel_stop, qos_wheel_stop, listener_);
    dds_delete_qos(qos_wheel_stop);

        if (reader_wheel_stop < 0) {
            DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_wheel_stop));
            return false;
        }
        spdlog::info("reader_wheel_stop created successfully");



    //急停指令
    topic_e_stop = dds_create_topic(
        participant_,
        &zdl_msg_dds__EStopRequest_desc,
        "zdl/msg/dds/request_ui_cmd/e_stop",
        NULL, NULL
    );
    if (topic_e_stop < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_e_stop));
        return false;
    }
    spdlog::info("topic_e_stop created successfully");

    dds_qos_t* qos_e_stop = dds_create_qos();
    dds_qset_reliability(qos_e_stop, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_durability(qos_e_stop, DDS_DURABILITY_VOLATILE);

    reader_e_stop = dds_create_reader(subscriber_, topic_e_stop, qos_e_stop, listener_);
    dds_delete_qos(qos_e_stop);

        if (reader_e_stop < 0) {
            DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_e_stop));
            return false;
        }
        spdlog::info("reader_e_stop created successfully");



    //CSP指令
    topic_csp_cmd = dds_create_topic(
        participant_,
        &zdl_msg_dds__CSPCommand_desc,
        "zdl/msg/dds/request_ui_cmd/csp_command",
        NULL, NULL
    );
    if (topic_csp_cmd < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_csp_cmd));
        return false;
    }

    spdlog::info("topic_csp_cmd created successfully");
    dds_qos_t* qos_csp_cmd = dds_create_qos();

    //  Best Effort（允许丢包，降低延迟）
    dds_qset_reliability(qos_csp_cmd, DDS_RELIABILITY_BEST_EFFORT, 0);

    // 不要历史数据
    dds_qset_durability(qos_csp_cmd, DDS_DURABILITY_VOLATILE);

    // 只保留最新一条
    dds_qset_history(qos_csp_cmd, DDS_HISTORY_KEEP_LAST, 1);

    //  限制资源（防止堆积）
    dds_qset_resource_limits(qos_csp_cmd, 1, 1, 1);
    
    reader_csp_cmd = dds_create_reader(subscriber_, topic_csp_cmd, qos_csp_cmd, listener_);
    dds_delete_qos(qos_csp_cmd);

        if (reader_csp_cmd < 0) {
            DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_csp_cmd));
            return false;
        }
        spdlog::info("reader_csp_cmd created successfully");



    //任务停止指令
    topic_stop_mission = dds_create_topic(
        participant_,
        &zdl_msg_dds__StopMissionRequest_desc,
        "zdl/msg/dds/request_ui_cmd/stop_mission",
        NULL, NULL
    );
    if (topic_stop_mission < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_stop_mission));
        return false;
    }
    spdlog::info("topic_stop_mission created successfully");

        dds_qos_t* qos_stop_mission = dds_create_qos();
    dds_qset_reliability(qos_stop_mission, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_durability(qos_stop_mission, DDS_DURABILITY_VOLATILE);

    reader_stop_mission = dds_create_reader(subscriber_, topic_stop_mission, qos_stop_mission, listener_);

    dds_delete_qos(qos_stop_mission);

        if (reader_stop_mission < 0) {
            DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader_stop_mission));
            return false;
        }
        spdlog::info("reader_stop_mission created successfully");






    //机器人状态
    topic_state_ = dds_create_topic(
    participant_,
    &zdl_msg_dds__state_RobotState_desc,
    "zdl/msg/dds/robot_state",
    NULL, NULL
    );
    if (topic_state_ < 0) {
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic_state_));
        return false;
    }


    dds_qos_t* qos_state = dds_create_qos();
    dds_qset_reliability(qos_state, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_durability(qos_state, DDS_DURABILITY_VOLATILE);

    writer_state_ = dds_create_writer(publisher_, topic_state_, qos_state, listener_);
    dds_delete_qos(qos_state);
    if (writer_state_ < 0) {
        DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer_state_));
        return false;
    }



    // dds_qos_t* qos_state = dds_create_qos();
    // dds_qset_reliability(qos_state, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(0));
    // dds_qset_durability(qos_state, DDS_DURABILITY_VOLATILE);

    // writer_state_ = dds_create_writer(publisher_, topic_state_, qos_state, NULL);
    // dds_delete_qos(qos_state);

    spdlog::info("writer_state created successfully");

        log_match_status();
        spdlog::info("InitDDS完成");
        return true;
    }



   
    

// 静态回调
void UiCmdServer::on_data_available(dds_entity_t reader, void* arg) {
    UiCmdServer* self = static_cast<UiCmdServer*>(arg);  // 获取对象实例
    self->handle_data(reader);
}

// 成员函数处理数据
void UiCmdServer::handle_data(dds_entity_t reader) {
    // spdlog::info("进入回调");

    if (reader == reader_start) {
        handle_start();
    }
    else if (reader == reader_stop) {
        handle_stop();
    }
    else if (reader == reader_enable){
        handle_enable();
    }
    else if (reader == reader_disable){
        handle_disable();
    }
    else if(reader == reader_set_zero_position){
        handle_set_zero_position();
    }
    else if(reader == reader_return_zero){
        handle_return_zero();
    }
    else if (reader == reader_jog_command)
    {
        handle_jog_command();
    }
    // else if (reader == reader_waist_target){
    //     handle_waist_target();
    // }
    // else if (reader == reader_wheel_stop){
    //     handle_wheel_stop();
    // }
    else if(reader == reader_e_stop){
        handle_e_stop();
    }
    else if(reader == reader_csp_cmd){
        handle_csp_command();
    }
    else if (reader == reader_stop_mission){
        handle_stop_mission();
    }
    else {
        spdlog::warn("未知的 reader 收到数据可用事件: {}", reader);
    }
    
}

void UiCmdServer::shutdownDds() {
    if (participant_ > 0) {
        dds_delete(participant_); // 级联删除子实体
        participant_ = DDS_ENTITY_NIL;
    }
    if (listener_) {
    dds_delete_listener(listener_);
    listener_ = nullptr;
}
}



void UiCmdServer::on_subscription_matched(
      dds_entity_t reader,
      const dds_subscription_matched_status_t status,
      void* arg) {
      UiCmdServer* self = static_cast<UiCmdServer*>(arg);
      if (self == nullptr) {
          return;
      }
      self->handle_subscription_matched(reader, status);
  }

    void UiCmdServer::handle_subscription_matched(
        dds_entity_t reader,
        const dds_subscription_matched_status_t& status)
    {
        if (reader == reader_start) {
            spdlog::info(
                "[START] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_stop) {
            spdlog::info(
                "[STOP] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_enable) {
            spdlog::info(
                "[ENABLE] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_disable) {
            spdlog::info(
                "[DISABLE] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_set_zero_position) {
            spdlog::info(
                "[SETZEROPOSITION] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_return_zero) {
            spdlog::info(
                "[RETURNZERO] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_jog_command) {
            spdlog::info(
                "[JOGCOMMAND] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_waist_target) {
            spdlog::info(
                "[WAIST_TARGET] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_wheel_stop) {
            spdlog::info(
                "[WHEEL_STOP] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if(reader == reader_e_stop){
            spdlog::info(
                "[E_STOP] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_csp_cmd) {
            spdlog::info(
                "[CSP_COMMAND] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else if (reader == reader_stop_mission) {
            spdlog::info(
                "[STOP_MISSION] 匹配变化 current={}, total={}",
                status.current_count,
                status.total_count);
        }
        else {
            spdlog::warn("未知的 reader 匹配事件: {}", reader);
        }
    }


  void UiCmdServer::log_match_status() const {

    //===== start topic =====
    if (reader_start <= DDS_ENTITY_NIL) {
        spdlog::warn("[START] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_start, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[START] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[START] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }

    // ===== stop topic =====
    if (reader_stop <= DDS_ENTITY_NIL) {
        spdlog::warn("[STOP] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_stop, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[STOP] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[STOP] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }

    // ===== enable topic =====
    if (reader_enable <= DDS_ENTITY_NIL) {
        spdlog::warn("[ENABLE] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_enable, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[ENABLE] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[ENABLE] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }

    // ===== disable topic =====
    if (reader_disable <= DDS_ENTITY_NIL) {
        spdlog::warn("[DISABLE] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_disable, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[DISABLE] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[DISABLE] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }
    // ===== setzeroposition topic =====
    if (reader_set_zero_position <= DDS_ENTITY_NIL) {
        spdlog::warn("[Setzeroposition] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_set_zero_position, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[Setzeroposition] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[Setzeroposition] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }


    // ===== returnzero topic =====
    if (reader_return_zero <= DDS_ENTITY_NIL) {
        spdlog::warn("[Returnzero] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_return_zero, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[Returnzero] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[Returnzero] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }
    // ===== jog_command topic =====
    if (reader_jog_command <= DDS_ENTITY_NIL) {
        spdlog::warn("[JogCommand] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_jog_command, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[JogCommand] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[JogCommand] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }



    // ===== waist_target topic =====
    if (reader_waist_target <= DDS_ENTITY_NIL) {
        spdlog::warn("[WAIST_TARGET] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_waist_target, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[WAIST_TARGET] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[WAIST_TARGET] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }




    // ===== wheel_stop topic =====
    if (reader_wheel_stop <= DDS_ENTITY_NIL) {
        spdlog::warn("[WHEEL_STOP] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_wheel_stop, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[WHEEL_STOP] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[WHEEL_STOP] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }



    // ===== e_stop topic =====
    if (reader_e_stop <= DDS_ENTITY_NIL) {
        spdlog::warn("[E_STOP] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_e_stop, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[E_STOP] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[E_STOP] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }




    // ===== csp_command topic =====
    if (reader_csp_cmd <= DDS_ENTITY_NIL) {
        spdlog::warn("[CSP_COMMAND] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_csp_cmd, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[CSP_COMMAND] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[CSP_COMMAND] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }





    // ==== stop_mission topic =====
    if (reader_stop_mission <= DDS_ENTITY_NIL) {
        spdlog::warn("[STOP_MISSION] reader 尚未初始化，无法查询匹配状态");
    } else {
        dds_subscription_matched_status_t st{};
        dds_return_t rc = dds_get_subscription_matched_status(reader_stop_mission, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[STOP_MISSION] dds_get_subscription_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[STOP_MISSION] 匹配状态: current={}, total={}, current_change={}, total_change={}",
                st.current_count,
                st.total_count,
                st.current_count_change,
                st.total_count_change);
        }
    }



    // ===== writer_state (机器人状态发布者) =====
    if (writer_state_ <= DDS_ENTITY_NIL) {
        spdlog::warn("[ROBOT_STATE] writer 尚未初始化，无法查询匹配状态");
    } else {
        dds_publication_matched_status_t st{};
        // 注意：Writer 使用的是 publication 相关的函数
        dds_return_t rc = dds_get_publication_matched_status(writer_state_, &st);

        if (rc != DDS_RETCODE_OK) {
            spdlog::error("[ROBOT_STATE] dds_get_publication_matched_status failed: {}", 
                          dds_strretcode(-rc));
        } else {
            spdlog::info(
                "[ROBOT_STATE] 匹配状态: current_readers={}, total_readers={}",
                st.current_count,
                st.total_count);
            
            if (st.current_count == 0) {
                spdlog::warn("[ROBOT_STATE] 当前没有任何订阅者(Reader)连接！UI端可能无法收到数据。");
            }
        }
    }

    // ===== 检查 Robot State Writer (重点排查对象) =====
    if (writer_state_ <= DDS_ENTITY_NIL) {
        spdlog::warn("[ROBOT_STATE_WRITER] 尚未初始化");
    } else {
        dds_publication_matched_status_t st{};
        dds_return_t rc = dds_get_publication_matched_status(writer_state_, &st);
        
        if (rc == DDS_RETCODE_OK) {
            spdlog::info("[ROBOT_STATE_WRITER] 匹配 Reader 数量: {}", st.current_count);
            if (st.current_count == 0) {
                spdlog::warn("[ROBOT_STATE_WRITER] 警告: 当前没有 UI 订阅者连接，数据将无法送达！");
            }
        } else {
            spdlog::error("[ROBOT_STATE_WRITER] 获取匹配状态失败: {}", dds_strretcode(-rc));
        }
    }
}




void UiCmdServer::handle_start()
{
    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__StartRequest__alloc();
    dds_return_t rc = DDS_RETCODE_OK;

    while ((rc = dds_take(reader_start, samples, infos, 1, 1)) > 0)
    {
        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__StartRequest*>(samples[0]);

        spdlog::info("start command");
        spdlog::info("========== 收到 START 指令 ==========");
        spdlog::info("request_id={}", msg->request_id);

        controller::StartConfig config;

        const std::string config_base = "/home/root/workspace/zdl-controller-toolkit/config";
        const std::string config_yaml_path = config_base + "/config.yaml";

        try {
            YAML::Node cfg = YAML::LoadFile(config_yaml_path);
            int current_idx = cfg["current_config"].as<int>();

            const YAML::Node& configs = cfg["config"];
            bool found = false;
            for (const auto& item : configs) {
                if (item["idx"].as<int>() == current_idx) {
                    std::string master_file = item["config_master_dir"].as<std::string>();
                    std::string device_file = item["config_device_dir"].as<std::string>();
                    config.master_yaml_path = config_base + "/" + master_file;
                    config.device_yaml_path = config_base + "/" + device_file;
                    spdlog::info("从 config.yaml 读取配置 idx={}, master={}, device={}",
                                 current_idx, master_file, device_file);
                    found = true;
                    break;
                }
            }
            if (!found) {
                spdlog::error("config.yaml 中未找到 idx={} 的配置，使用默认值", current_idx);
                config.master_yaml_path = config_base + "/config_single_motor_master.yaml";
                config.device_yaml_path = config_base + "/config_single_motor_device.yaml";
            }
        } catch (const std::exception& e) {
            spdlog::error("读取 config.yaml 失败: {}，使用默认配置", e.what());
            config.master_yaml_path = config_base + "/config_single_motor_master.yaml";
            config.device_yaml_path = config_base + "/config_single_motor_device.yaml";
        }
        config.urdf_path = "/home/root/workspace/zdl-controller-toolkit/data/duo_arm.urdf";

        
    config.left_arm_direction = {true, true, true, true, true, true, true};
    config.right_arm_direction = {true, true, true, true, true, true, true};
      

        config.left_arm_type = controller::ArmType::k7Dof;
        config.right_arm_type = controller::ArmType::k7Dof;
        config.left_hand_type = controller::HandType::kO7;
        config.right_hand_type = controller::HandType::kO7;
        config.head_type = controller::HeadType::kEnable;
        config.waist_type = controller::WaistType::kLiftWaist;
        config.wheel_type = controller::WheelType::kDoubleWheel;

        // 6) part_config 解析
        config.enable_error_code_check = false;
        config.enable_csp_output_limit = false;
        config.enable_encoder_check = false;
      
        // 7) 调用 robot_->connect
        if (!robot_) {
            spdlog::error("robot 未初始化");
        } else {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->connect(config);
        }

        spdlog::info("=================================");
    }

    if (rc < 0) {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__StartRequest_free(samples[0], DDS_FREE_ALL);
}


void UiCmdServer::handle_stop() {

    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__StopRequest__alloc();

    dds_return_t rc;

    while ((rc = dds_take(reader_stop, samples, infos, 1, 1)) > 0) {

        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__StopRequest*>(samples[0]);

        spdlog::info("stop command");

        // spdlog::info("========== 收到 STOP 指令 ==========");
        // spdlog::info("request_id={}", msg->request_id);
        
        if (robot_) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->disconnect();
        }
        else {
            spdlog::error("robot 未初始化");
        }

 
    }

    if (rc < 0) {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__StopRequest_free(samples[0], DDS_FREE_ALL);
}


void UiCmdServer::handle_enable() 
{

    //spdlog::info("enable command");

    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__EnableRequest__alloc();

    dds_return_t rc;

    while ((rc = dds_take(reader_enable, samples, infos, 1, 1)) > 0) {

        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__EnableRequest*>(samples[0]);

        //spdlog::info("========== 收到 ENABLE 指令 ==========");
       // spdlog::info("request_id={}", msg->request_id);

        bool valid = true;

        switch (msg->request_mode) {

            case zdl_msg_dds__kCSP:
                spdlog::info("使能模式: CSP (位置模式)");
                break;

            case zdl_msg_dds__kCST:
                spdlog::info("使能模式: CST (力矩模式)");
                break;

            default:
                spdlog::error("未知 enable 模式: {}", (int)msg->request_mode);
                valid = false;
                break;
        }

        if (valid) {

            if (!robot_) {
                spdlog::error("robot 未初始化");
            }
            else {
               
                std::lock_guard<std::mutex> lock(robot_mutex_);

                // 这里调用你的机器人控制接口
                robot_->enable(controller::RequestMode::kCSP);

                spdlog::info("robot enable 执行完成");
                
                
       

            }

    if (rc < 0) {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__EnableRequest_free(samples[0], DDS_FREE_ALL);
    }
}

}



void UiCmdServer::handle_disable(){
    //spdlog::info("disable command");

    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__DisableRequest__alloc();

    dds_return_t rc;

    while ((rc = dds_take(reader_disable, samples, infos, 1, 1)) > 0) {

        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__DisableRequest*>(samples[0]);

        //spdlog::info("========== 收到 Disable 指令 ==========");
        //spdlog::info("request_id={}", msg->request_id);
        
        if (robot_) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->disable();
        }
        else {
            spdlog::error("robot 未初始化");
        }

       
    }

    if (rc < 0) {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__DisableRequest_free(samples[0], DDS_FREE_ALL);
}



void UiCmdServer::handle_set_zero_position(){
    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__SetZeroPositionRequest__alloc();

    dds_return_t rc;

    while ((rc = dds_take(reader_set_zero_position, samples, infos, 1, 1)) > 0) {

        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__SetZeroPositionRequest*>(samples[0]);

        spdlog::info("set_zero_position command");
        spdlog::info("========== 收到 Set_zero_position 指令 ==========");
        spdlog::info("request_id={}", msg->request_id);
        
        if (robot_) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->resetZeroPosition();
        }
        else {
            spdlog::error("robot 未初始化");
        }

        spdlog::info("=================================");
    }

    if (rc < 0) {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__SetZeroPositionRequest_free(samples[0], DDS_FREE_ALL);
}


void UiCmdServer::handle_return_zero(){
    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__ReturnZeroRequest__alloc();

    dds_return_t rc;

    while ((rc = dds_take(reader_return_zero, samples, infos, 1, 1)) > 0) {

        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__ReturnZeroRequest*>(samples[0]);

        spdlog::info("return_zero command");
        auto state = robot_->getRobotState();
        spdlog::info("当前状态：{}",static_cast<int>(state.current_mission));
        spdlog::info("当前模式：{}",static_cast<int>(state.current_mode));
        spdlog::info("========== 收到 Return_zero 指令 ==========");
        spdlog::info("request_id={}", msg->request_id);
        
        if (robot_) {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->returnToZero();
        }
        else {
            spdlog::error("robot 未初始化");
        }

        spdlog::info("=================================");
    }

    if (rc < 0) {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__ReturnZeroRequest_free(samples[0], DDS_FREE_ALL);
    exportInterpolatedPositions("/home/root/workspace/zdl-controller-toolkit/interpolated_positions.csv");
}




void UiCmdServer::handle_jog_command()
{
    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__JogCommand__alloc();

    dds_return_t rc;

    while ((rc = dds_take(reader_jog_command, samples, infos, 1, 1)) > 0)
    {
        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__JogCommand*>(samples[0]);

        spdlog::info("jog_command received");
        spdlog::info("========== 收到 Jog Command ==========");

        int joint_id = msg->joint_id;
        auto joint = static_cast<controller::JointSelect>(joint_id);

        controller::Direction dir;

        if (!msg->active)
        {
            // 停止
            dir = controller::Direction::kStop;
            spdlog::info("Jog STOP");
        }
        else
        {
            if (msg->direction == zdl_msg_dds__FORWARD)
            {
                dir = controller::Direction::kForward;
                spdlog::info("Jog FORWARD");
            }
            else
            {
                dir = controller::Direction::kReverse;
                spdlog::info("Jog REVERSE");
            }
        }

        spdlog::info("Joint: {}", joint_id);

        if (robot_)
        {
            spdlog::info("enter joggggggggg");
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->jogControl(joint, dir);
        }
        else
        {
            spdlog::error("robot 未初始化");
        }

        spdlog::info("=================================");
    }

    if (rc < 0)
    {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__JogCommand_free(samples[0], DDS_FREE_ALL);
}




// void UiCmdServer::handle_waist_target(){
//     spdlog::info("waist_target received");

//     void* samples[1];
//     dds_sample_info_t infos[1];

//     samples[0] = zdl_msg_dds__WaistTarget__alloc();

//     dds_return_t rc;

//     while ((rc = dds_take(reader_waist_target, samples, infos, 1, 1)) > 0)
//     {
//         if (!infos[0].valid_data)
//             continue;

//         auto* msg = static_cast<zdl_msg_dds__WaistTarget*>(samples[0]);

//         spdlog::info("========== 收到 Return_zero 指令 ==========");
//         spdlog::info("request_id={}", msg->request_id);

//         if (robot_)
//         {
//             std::lock_guard<std::mutex> lock(robot_mutex_);
//             robot_->setFoldedWaistTarget(msg->target);
//         }
//         else
//         {
//             spdlog::error("robot 未初始化");
//         }

//         spdlog::info("=================================");
//     }

//     if (rc < 0)
//     {
//         spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
//     }

//     zdl_msg_dds__WaistTarget_free(samples[0], DDS_FREE_ALL);
// }




// void UiCmdServer::handle_wheel_stop(){
//     spdlog::info("wheel_stop received");

//     void* samples[1];
//     dds_sample_info_t infos[1];

//     samples[0] = zdl_msg_dds__WheelStop__alloc();

//     dds_return_t rc;

//     while ((rc = dds_take(reader_wheel_stop, samples, infos, 1, 1)) > 0)
//     {
//         if (!infos[0].valid_data)
//             continue;

//         auto* msg = static_cast<zdl_msg_dds__WheelStop*>(samples[0]);

//         spdlog::info("========== 收到 Wheel Stop 指令 ==========");
//         spdlog::info("request_id={}", msg->request_id);

//         if (robot_)
//         {
//             std::lock_guard<std::mutex> lock(robot_mutex_);
//             robot_->setWheelTarget(0.0,0.0);
//         }
//         else
//         {
//             spdlog::error("robot 未初始化");
//         }

//         spdlog::info("=================================");
//     }

//     if (rc < 0)
//     {
//         spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
//     }

//     zdl_msg_dds__WheelStop_free(samples[0], DDS_FREE_ALL);
// }



void UiCmdServer::publishRobotState() {

    if (!robot_) return;
    if (writer_state_ <= DDS_ENTITY_NIL) return;
    controller::RobotState state;

    {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        state = robot_->getRobotState();
    }

    zdl_msg_dds__state_RobotState msg;
    memset(&msg, 0, sizeof(msg));

   
    ToDdsRobotState(state, msg);







    

    
    // -----------------------

    dds_return_t rc = dds_write(writer_state_, &msg);

    if (rc != DDS_RETCODE_OK) {
        spdlog::error("dds_write failed: {}", dds_strretcode(-rc));
    }

   
}


void UiCmdServer::on_publication_matched(
    dds_entity_t writer,
    const dds_publication_matched_status_t status,
    void* arg) {
    UiCmdServer* self = static_cast<UiCmdServer*>(arg);
    if (self == nullptr) return;
    self->handle_publication_matched(writer, status);
}



void UiCmdServer::handle_publication_matched(
    dds_entity_t writer,
    const dds_publication_matched_status_t& status) 
{
    if (writer == writer_state_) {
        spdlog::info(
            "[ROBOT_STATE_WRITER] 匹配变化: 当前接收者(Readers)={}, 累计连接过={}",
            status.current_count,
            status.total_count);
        
        if (status.current_count == 0) {
            spdlog::warn("[ROBOT_STATE_WRITER] 失去所有订阅者，UI 端可能已离线。");
        } else {
            spdlog::info("[ROBOT_STATE_WRITER] 检测到 UI 端上线，开始同步状态数据。");
        }
    }
}


void UiCmdServer::handle_e_stop(){
    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__EStopRequest__alloc();

    dds_return_t rc;

    while ((rc = dds_take(reader_e_stop, samples, infos, 1, 1)) > 0)
    {
        if (!infos[0].valid_data)
            continue;

        auto* msg = static_cast<zdl_msg_dds__EStopRequest*>(samples[0]);

        spdlog::info("e_stop received");
        spdlog::info("========== 收到 E-STOP 指令 ==========");
        spdlog::info("request_id={}", msg->request_id);

        if (robot_)
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            robot_->emergencyStop();
        }
        else
        {
            spdlog::error("robot 未初始化");
        }

        spdlog::info("=================================");
    }

    if (rc < 0)
    {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__EStopRequest_free(samples[0], DDS_FREE_ALL);
}


void UiCmdServer::handle_csp_command()
{
    void* samples[8];
    dds_sample_info_t infos[8];

    for (int i = 0; i < 8; ++i)
    {
        samples[i] = zdl_msg_dds__CSPCommand__alloc();
    }

    dds_return_t rc;

    while ((rc = dds_take(reader_csp_cmd, samples, infos, 8, 8)) > 0)
    {
        for (int i = 0; i < rc; ++i)
        {
            if (!infos[i].valid_data)
                continue;

            auto* msg = static_cast<zdl_msg_dds__CSPCommand*>(samples[i]);

            if (robot_)
            {
                if (!csp_running_)
                {
                    if (csp_thread_.joinable())
                        csp_thread_.join();
                    csp_running_ = true;
                    csp_thread_ = std::thread([this]() {
                        startCSPControl();
                    });
                }

                interpolator_.receive(msg->timestamp, std::array<double,36>{
                    msg->joint[0],  msg->joint[1],  msg->joint[2],  msg->joint[3],
                    msg->joint[4],  msg->joint[5],  msg->joint[6],  msg->joint[7],
                    msg->joint[8],  msg->joint[9],  msg->joint[10], msg->joint[11],
                    msg->joint[12], msg->joint[13], msg->joint[14], msg->joint[15],
                    msg->joint[16], msg->joint[17], msg->joint[18], msg->joint[19],
                    msg->joint[20], msg->joint[21], msg->joint[22], msg->joint[23],
                    msg->joint[24], msg->joint[25], msg->joint[26], msg->joint[27],
                    msg->joint[28], msg->joint[29], msg->joint[30], msg->joint[31],
                    msg->joint[32], msg->joint[33], msg->joint[34], msg->joint[35]
                });
            }
            else
            {
                spdlog::error("robot 未初始化");
            }
        }
    }

    if (rc < 0)
    {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    for (int i = 0; i < 8; ++i)
    {
        zdl_msg_dds__CSPCommand_free(samples[i], DDS_FREE_ALL);
    }
}





void UiCmdServer::startCSPControl()
{
    spdlog::info("before runCycleJointMotion");
    auto cmd_callback =
      [this](const controller::RobotState& s, controller::Duration time, controller::JointPositionCmd& cmd)
    {
        if (!cmd.isInitialized())
        {
        cmd.enableRightArm();
        cmd.enableRightHand();
        cmd.enableLeftArm();
        cmd.enableLeftHand();
        // cmd.enableHead();
   
        
        // cmd.enableWaistPP();
        cmd.enableWheel();
        cmd.setInitialized();
        }
        std::array<double, 36> q_interp;
        bool ok = interpolator_.get(q_interp);
        if (ok)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            interpolated_positions_.push_back(q_interp);
                // spdlog::info("存储了");
        }
        if (!ok)
        {
            spdlog::warn("插值器无数据，返回当前关节位置");
            
            for (int i = 0; i < 7; ++i)
            {
                cmd.right_arm.q_d[i] = s.right_arm.q[i];
                cmd.left_arm.q_d[i]  = s.left_arm.q[i];
                cmd.right_hand.q_d[i] = s.right_hand.q[i];
                cmd.left_hand.q_d[i] = s.left_hand.q[i];
                
            }
            cmd.waist.q_d[0] = s.folded_waist.q[0];
            cmd.wheel.dq_d = {0.0, 0.0};
            return;
        }

        cmd.right_arm.q_d[0] = q_interp[0];
        cmd.right_arm.q_d[1] = q_interp[1];
        cmd.right_arm.q_d[2] = q_interp[2];
        cmd.right_arm.q_d[3] = q_interp[3];
        cmd.right_arm.q_d[4] = q_interp[4];
        cmd.right_arm.q_d[5] = q_interp[5];
        cmd.right_arm.q_d[6] = q_interp[6];

        cmd.right_hand.q_d[0] = q_interp[7];
        cmd.right_hand.q_d[1] = q_interp[8];
        cmd.right_hand.q_d[2] = q_interp[9];
        cmd.right_hand.q_d[3] = q_interp[10];
        cmd.right_hand.q_d[4] = q_interp[11];
        cmd.right_hand.q_d[5] = q_interp[12];
        cmd.right_hand.q_d[6] = q_interp[13];

        cmd.right_hand.dq_d[0] = 100.0;
        cmd.right_hand.dq_d[1] = 100.0;
        cmd.right_hand.dq_d[2] = 100.0;
        cmd.right_hand.dq_d[3] = 100.0;
        cmd.right_hand.dq_d[4] = 100.0;
        cmd.right_hand.dq_d[5] = 100.0;
        cmd.right_hand.dq_d[6] = 100.0;

        cmd.right_hand.tau_d[0] = 1.0;
        cmd.right_hand.tau_d[1] = 100.0;
        cmd.right_hand.tau_d[2] = 100.0;
        cmd.right_hand.tau_d[3] = 100.0;
        cmd.right_hand.tau_d[4] = 100.0;
        cmd.right_hand.tau_d[5] = 100.0;
        cmd.right_hand.tau_d[6] = 100.0;

        cmd.left_arm.q_d[0] = q_interp[14];
        cmd.left_arm.q_d[1] = q_interp[15];
        cmd.left_arm.q_d[2] = q_interp[16];
        cmd.left_arm.q_d[3] = q_interp[17];
        cmd.left_arm.q_d[4] = q_interp[18];
        cmd.left_arm.q_d[5] = q_interp[19];
        cmd.left_arm.q_d[6] = q_interp[20];
        cmd.left_hand.q_d[0] = q_interp[21];
        cmd.left_hand.q_d[1] = q_interp[22];
        cmd.left_hand.q_d[2] = q_interp[23];
        cmd.left_hand.q_d[3] = q_interp[24];
        cmd.left_hand.q_d[4] = q_interp[25];
        cmd.left_hand.q_d[5] = q_interp[26];
        cmd.left_hand.q_d[6] = q_interp[27];

        cmd.left_hand.dq_d[0] = 100.0;
        cmd.left_hand.dq_d[1] = 100.0;
        cmd.left_hand.dq_d[2] = 100.0;
        cmd.left_hand.dq_d[3] = 100.0;
        cmd.left_hand.dq_d[4] = 100.0;
        cmd.left_hand.dq_d[5] = 100.0;
        cmd.left_hand.dq_d[6] = 100.0;

        cmd.left_hand.tau_d[0] = 1.0;
        cmd.left_hand.tau_d[1] = 100.0;
        cmd.left_hand.tau_d[2] = 100.0;
        cmd.left_hand.tau_d[3] = 100.0;
        cmd.left_hand.tau_d[4] = 100.0;
        cmd.left_hand.tau_d[5] = 100.0;
        cmd.left_hand.tau_d[6] = 100.0;
        
        cmd.waist.q_d[0] = q_interp[30];
	// cmd.head.q_d[0] = q_interp[28];
    //     cmd.head.q_d[1] = q_interp[29];
        cmd.wheel.dq_d = {q_interp[34],q_interp[35]};
        
    };
    

    robot_->runCycleJointMotion(cmd_callback); 
    // csp_running_ = false;
    spdlog::info("after runCycleJointMotion");
}



void UiCmdServer::exportInterpolatedPositions(const std::string& filename)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open())
    {
        spdlog::error("无法打开文件 {}", filename);
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    for (const auto& pos : interpolated_positions_)
    {
        for (size_t i = 0; i < pos.size(); ++i)
        {
            ofs << pos[i];
            if (i != pos.size() - 1) ofs << ",";
        }
        ofs << "\n";
    }

    spdlog::info("已导出插值角度数据到 {}", filename);
}




void UiCmdServer::handle_stop_mission() {
    void* samples[1];
    dds_sample_info_t infos[1];

    samples[0] = zdl_msg_dds__StopMissionRequest__alloc();
    if (!samples[0]) {
        spdlog::error("StopMissionRequest alloc failed");
        return;
    }

    dds_return_t rc = DDS_RETCODE_OK;

    while ((rc = dds_take(reader_stop_mission, samples, infos, 1, 1)) > 0) {
        if (!infos[0].valid_data) {
            continue;
        }

        auto* msg = static_cast<zdl_msg_dds__StopMissionRequest*>(samples[0]);

        spdlog::info("stop_mission received");
        spdlog::info("========== 收到 Stop Mission 指令 ==========");
        spdlog::info("request_id={}", msg->request_id);

        if (robot_) {
            {
                std::lock_guard<std::mutex> lock(robot_mutex_);
                robot_->stopCurrentMission();
            }
            csp_running_ = false;  // 若多线程访问，建议改成 atomic<bool>
        } else {
            spdlog::error("robot 未初始化");
        }

        spdlog::info("=================================");
    }

    if (rc < 0) {
        spdlog::error("dds_take failed: {}", dds_strretcode(-rc));
    }

    zdl_msg_dds__StopMissionRequest_free(samples[0], DDS_FREE_ALL);
}
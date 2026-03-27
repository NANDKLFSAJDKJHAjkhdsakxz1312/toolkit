#include "dds/dds.h"
#include "idl/start_task.h"
#include <spdlog/spdlog.h>
#include "utils/struct_init.h"
#include "ZdlController/controller_api.h"
#include "idl/stop_task.h"
#include "idl/enable.h"
#include "idl/disable.h"
#include "idl/set_zero_position.h"
#include "idl/return_zero.h"
#include "include/motion_data_loader.h"
#include "include/csv_saver.hpp"
#include "idl/jog_command.h"
#include "idl/robot_state.h"
#include "utils/state_conversion_to_dds.h"
#include "idl/waist_target.h"
#include "idl/wheel_stop.h"
#include "idl/estop.h"
#include "idl/stop_mission.h"
#include "linear_interpolator.h"
#include "idl/csp_command.h"


class UiCmdServer {
public:
    UiCmdServer();
    ~UiCmdServer();

    // 其他成员函数
    bool initDds();
    void shutdownDds();

    // 静态回调函数
    static void on_data_available(dds_entity_t reader, void* arg);
    static void on_subscription_matched(
        dds_entity_t reader,
        const dds_subscription_matched_status_t status,
        void* arg);

    static void on_publication_matched(dds_entity_t writer,
        const dds_publication_matched_status_t status,
        void* arg);
        void handle_publication_matched(dds_entity_t writer,
        const dds_publication_matched_status_t& status);


    void handle_data(dds_entity_t reader);
    void handle_subscription_matched(
        dds_entity_t reader,
        const dds_subscription_matched_status_t& status);

    void log_match_status() const;

    void handle_start();
    void handle_stop();
    void handle_enable();
    void handle_disable();
    void handle_set_zero_position();
    void handle_return_zero();
    void handle_jog_command();
    void handle_waist_target();
    void publishRobotState();
    void handle_wheel_stop();
    void handle_e_stop();
    void handle_csp_command();
    void startCSPControl();
    void handle_stop_mission();


    void exportInterpolatedPositions(const std::string& filename);
private:

    std::unique_ptr<controller::ControllerInterface> robot_;
    dds_entity_t participant_ = DDS_ENTITY_NIL;
    dds_entity_t subscriber_ = DDS_ENTITY_NIL;
    dds_entity_t topic_start = DDS_ENTITY_NIL;
    dds_entity_t reader_start = DDS_ENTITY_NIL;
    dds_entity_t topic_stop = DDS_ENTITY_NIL;
    dds_entity_t reader_stop = DDS_ENTITY_NIL;
    dds_entity_t topic_enable = DDS_ENTITY_NIL;
    dds_entity_t reader_enable = DDS_ENTITY_NIL;
    dds_entity_t topic_disable = DDS_ENTITY_NIL;
    dds_entity_t reader_disable = DDS_ENTITY_NIL;
    dds_entity_t topic_set_zero_position = DDS_ENTITY_NIL;
    dds_entity_t reader_set_zero_position = DDS_ENTITY_NIL;
    dds_entity_t topic_return_zero = DDS_ENTITY_NIL;
    dds_entity_t reader_return_zero = DDS_ENTITY_NIL;
    dds_entity_t topic_jog_command = DDS_ENTITY_NIL;
    dds_entity_t reader_jog_command = DDS_ENTITY_NIL;
    dds_entity_t topic_waist_target = DDS_ENTITY_NIL;
    dds_entity_t reader_waist_target = DDS_ENTITY_NIL;
    dds_entity_t topic_wheel_stop = DDS_ENTITY_NIL;
    dds_entity_t reader_wheel_stop = DDS_ENTITY_NIL;
    dds_entity_t topic_e_stop = DDS_ENTITY_NIL;
    dds_entity_t reader_e_stop = DDS_ENTITY_NIL;
    dds_entity_t topic_stop_mission = DDS_ENTITY_NIL;
    dds_entity_t reader_stop_mission = DDS_ENTITY_NIL;
    dds_listener_t* listener_ = nullptr;
    std::mutex robot_mutex_;



    dds_entity_t publisher_ = DDS_ENTITY_NIL;
    dds_entity_t topic_state_ = DDS_ENTITY_NIL;
    dds_entity_t writer_state_ = DDS_ENTITY_NIL;

    std::thread state_thread_;
    std::atomic<bool> running_{true};



    LinearInterpolator interpolator_;
    dds_entity_t reader_csp_cmd = DDS_ENTITY_NIL;
    dds_entity_t topic_csp_cmd = DDS_ENTITY_NIL;
    bool csp_running_ = false;
    std::vector<std::array<double, 14>> interpolated_positions_;
    std::mutex data_mutex_; 
};

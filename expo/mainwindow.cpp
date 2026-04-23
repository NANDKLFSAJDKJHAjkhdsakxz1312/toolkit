#include "mainwindow.h"

#include <QDebug>
#include <QElapsedTimer>
#include <QHeaderView>
#include <QTableWidget>
#include <algorithm>
#include <cmath>
#include <thread>

#include "./ui_mainwindow.h"
#include "motion_data_loader.h"

// mainwindow.cpp
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), status_update_timer_(std::make_unique<QTimer>(this))
{
  ui->setupUi(this);

  // 设置 QLabel 属性
  ui->icon->setMinimumSize(150, 50);
  ui->icon->setScaledContents(true);
  QPixmap pix(":/images/icon.png");
  if (!pix.isNull())
  {
    ui->icon->setPixmap(pix);
  }

  connect(status_update_timer_.get(), &QTimer::timeout, this, &MainWindow::updateUI);
  status_update_timer_->start(200);  // 每 200ms 更新一次（5Hz）

  initStatusTable();

  robot_ = controller::createController();
}

MainWindow::~MainWindow()
{
  robot_->emergencyStop();
  delete ui;
}

void MainWindow::initStatusTable()
{
  auto init_status_table = [&](QTableWidget* statusTable, double count, QStringList& headers) {
    if (!statusTable) return;

    // 设置表格行列数
    statusTable->setRowCount(count);
    statusTable->setColumnCount(headers.size());
    statusTable->setHorizontalHeaderLabels(headers);

    // 设置表头行为适应内容
    statusTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    statusTable->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    statusTable->setStyleSheet("QTableWidget { font-size: 9pt; }");

    // 初始化行标题（关节 ID）
    for (int i = 0; i < count; ++i)
    {
      QTableWidgetItem* item = new QTableWidgetItem(QString::number(i));
      statusTable->setVerticalHeaderItem(i, item);
    }

    // 初始化所有单元格
    for (int row = 0; row < count; ++row)
    {
      for (int col = 0; col < headers.size(); ++col)
      {
        QTableWidgetItem* item = new QTableWidgetItem("");
        statusTable->setItem(row, col, item);
        item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
      }
    }
  };

  // 设置表头标签
  QStringList headers;
  headers << "关节位置" << "关节速度" << "电机力矩" << "传感器力矩" << "状态字" << "运行模式";

  // QStringList headers2;
  // headers2 << "关节位置" << "期望位置" << "位置偏差" << "关节速度" << "电机力矩" << "状态字";

  init_status_table(ui->statusTable_right, 7, headers);
  init_status_table(ui->statusTable_left, 7, headers);
  // init_status_table(ui->statusTable_waist, 4, headers2);
}

void MainWindow::updateStatusTable()
{
  auto state = robot_->getRobotState();

  enum class ArmSelect { kLeft, kRight };

  auto update_status_table = [&](QTableWidget* statusTable, ArmSelect arm) {
    controller::RobotState::ArmState current_arm;
    if (arm == ArmSelect::kLeft)
    {
      current_arm = state.left_arm;
    }
    else
    {
      current_arm = state.right_arm;
    }

    for (auto i = 0; i < 7; ++i)
    {
      statusTable->item(i, 0)->setText(QString::number(current_arm.q.at(i), 'f', 3));

      statusTable->item(i, 1)->setText(QString::number(current_arm.dq.at(i), 'f', 3));

      statusTable->item(i, 2)->setText(QString::number(current_arm.tau_J.at(i), 'f', 3));

      statusTable->item(i, 3)->setText(QString::number(current_arm.tau_Js.at(i), 'f', 3));

      statusTable->item(i, 4)->setText(QString("0x%1").arg(current_arm.status_word.at(i), 4, 16, QChar('0')));

      statusTable->item(i, 5)->setText(QString::number(current_arm.mode_of_operation.at(i)));
    }
  };

  update_status_table(ui->statusTable_right, ArmSelect::kRight);
  update_status_table(ui->statusTable_left, ArmSelect::kLeft);
}

void MainWindow::updateUI()
{
  auto state = robot_->getRobotState();
  switch (state.current_mode)
  {
    case controller::CurrentMode::kNotStart:
      ui->current_mode->setText("任务未启动");
      break;

    case controller::CurrentMode::kNotEnable:
      ui->current_mode->setText("未使能");
      break;

    case controller::CurrentMode::kCSP:
      ui->current_mode->setText("CSP 使能");
      break;

    case controller::CurrentMode::kCST:
      ui->current_mode->setText("CST 使能");
      break;

    default:
      break;
  }

  switch (state.current_mission)
  {
    case controller::CurrentMission::kUnavailable:
      ui->current_mission->setText("不可用");
      break;

    case controller::CurrentMission::kHoming:
      ui->current_mission->setText("标零中");
      break;

    case controller::CurrentMission::kIdle:
      ui->current_mission->setText("空闲");
      break;

    case controller::CurrentMission::kJog:
      ui->current_mission->setText("点动");
      break;

    case controller::CurrentMission::kJointImpedance:
      ui->current_mission->setText("关节阻抗");
      break;

    case controller::CurrentMission::kCartImpedance:
      ui->current_mission->setText("笛卡尔阻抗");
      break;

    case controller::CurrentMission::kDrag:
      ui->current_mission->setText("零力拖动");
      break;

    case controller::CurrentMission::kReturnToZero:
      ui->current_mission->setText("回零中");
      break;

    case controller::CurrentMission::kUserCSP:
      ui->current_mission->setText("kUserCSP");
      break;

    case controller::CurrentMission::kUserCST:
      ui->current_mission->setText("kUserCST");
      break;

    case controller::CurrentMission::kFoldedWaistControl:
      ui->current_mission->setText("FWC");
      break;

    default:
      ui->current_mission->setText("???");
      break;
  }

  switch (state.current_errors)
  {
    case controller::RobotError::OK:
      ui->current_error->setText("");
      break;

    case controller::RobotError::kEmergencyStop:
      ui->current_error->setText("急停触发");
      break;

    default:
      break;
  }

  updateStatusTable();
  // double target = static_cast<double>(ui->waist_target_slider->value()) / 1000.0;
  // ui->waist_target_show->setText(QString::number(target, 'f', 3));

  // const double left_wheel_target = static_cast<double>(ui->left_wheel_target_slider->value()) / 100.0;
  // const double right_wheel_target = static_cast<double>(ui->right_wheel_target_slider->value()) / 100.0;
  // ui->left_wheel_target_show->setText(QString::number(left_wheel_target, 'f', 3));
  // ui->right_wheel_target_show->setText(QString::number(right_wheel_target, 'f', 3));

  // ui->hand_target_show->setText(QString::number(ui->hand_target_slider->value()));
}

void MainWindow::on_btn_e_stop_clicked()
{
  robot_->emergencyStop();
}

void MainWindow::on_btn_s1_clicked()
{
  controller::StartConfig config;
  if (ui->btn_left_only->isChecked())
  {
    config.arm_mode = controller::CurrentArmMode::kLeftArmOnly;
  }
  else if (ui->btn_right_only->isChecked())
  {
    config.arm_mode = controller::CurrentArmMode::kRightArmOnly;
  }
  else if (ui->btn_duo->isChecked())
  {
    config.arm_mode = controller::CurrentArmMode::kDualArm;
  }
  else if (ui->btn_waist_only->isChecked())
  {
    config.arm_mode = controller::CurrentArmMode::kFoldedWaistOnly;
  }
  robot_->start(config);
}

void MainWindow::on_btn_s2_clicked()
{
  robot_->stop();
}

void MainWindow::on_btn_s3_clicked()
{
  controller::RequestMode mode;
  if (ui->radioButton_CSP->isChecked())
  {
    mode = controller::RequestMode::kCSP;
  }
  else if (ui->radioButton_CST->isChecked())
  {
    mode = controller::RequestMode::kCST;
  }
  robot_->enable(mode);
}

void MainWindow::on_btn_s4_clicked()
{
  robot_->disable();
}

void MainWindow::on_btn_s5_clicked()
{
  robot_->setZeroPosition();
}

void MainWindow::on_btn_f2_clicked()
{
  robot_->returnToZero();
}

void MainWindow::on_btn_forward_pressed()
{
  auto joint = static_cast<controller::JointSelect>(ui->comboBox->currentIndex());
  robot_->jogControl(joint, controller::Direction::kForward);
}

void MainWindow::on_btn_forward_released()
{
  auto joint = static_cast<controller::JointSelect>(ui->comboBox->currentIndex());
  robot_->jogControl(joint, controller::Direction::kStop);
}

void MainWindow::on_btn_reverse_pressed()
{
  auto joint = static_cast<controller::JointSelect>(ui->comboBox->currentIndex());
  robot_->jogControl(joint, controller::Direction::kReverse);
}

void MainWindow::on_btn_reverse_released()
{
  auto joint = static_cast<controller::JointSelect>(ui->comboBox->currentIndex());
  robot_->jogControl(joint, controller::Direction::kStop);
}

void MainWindow::on_btn_f4_clicked()
{
  auto torque_callback = [](const controller::RobotState& s,
                            controller::Duration time) -> controller::Torques {
    const double t = time.toSec();
    std::array<double, 14> tau_cmd{0.0};

    // 2. 正弦运动参数
    constexpr double A = 5.0;  // 振幅 5.0 N*m
    constexpr double T = 4.0;  // 周期 4.0 s
    constexpr double omega = 2.0 * M_PI / T;

    double q1_target = A * std::sin(omega * t);

    tau_cmd[0] = q1_target;

    if (time.toMSec() % 500 == 0)
    {
      printf("Time: %.2f s, Target Q1: %.3f\n", t, q1_target);
    }

    return controller::Torques(tau_cmd);
  };

  robot_->runCycleTorque(torque_callback);
}

void MainWindow::on_btn_f8_clicked()
{
  robot_->stopCurrentMisiion();
  // csv_saver_timer_->stop();
}

void MainWindow::on_btn_f9_clicked() 
{
  // // 先检查是否是 CSP 模式
  // if (!robot_->enable(controller::RequestMode::kCSP))
  // {
  //   spdlog::error("使能失败，请检查机器人状态");
  //   return;
  // }

  // initCsvSaver();

  // 指定运动数据文件路径
  motion_file_path_ =
      "../data/"
      "left_right_hand_wave20251124.txt";  // 修改为实际文件路径

  // 创建并启动 MotionDataLoader
  motion_loader_ = std::make_unique<toolkit::MotionDataLoader>();

  // 设置回调函数，用于从 motion_loader 获取数据
  motion_loader_->setMotionCallback([this](double timestamp) -> std::vector<double> {
    std::vector<double> positions;
    std::vector<double> velocities;
    double ts;
    if (motion_loader_->getLatestJointPositions(positions, velocities, ts))
    {
      return positions;
    }
    return std::vector<double>(toolkit::BODY_JOINTS_NUM, 0.0);
  });

  // 启动离线模式（从文件加载）
  motion_loader_->startOffline(motion_file_path_);

  // 等待数据准备好
  while (!motion_loader_->isDataReady() && motion_loader_->isRunning())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto motion_callback = [this](const controller::RobotState& s,
                                controller::Duration time) -> controller::JointPositions {
    controller::ControlCommand q_cmd;
    static int count = 0;
    // 从 motion_loader 获取计算出的数据
    std::vector<double> motion_positions;
    std::vector<double> motion_velocities;
    double motion_timestamp;
    if (motion_loader_ && motion_loader_->isDataReady())
    {
      motion_loader_->getLatestJointPositions(motion_positions, motion_velocities, motion_timestamp);
    }

    for (int i = 0; i < 7; ++i)
    {
      q_cmd.right_arm[i] = toolkit::kJointDirectionFlag[i] ? motion_positions[i] * 2 * M_PI / 360.0
                                                           : -motion_positions[i] * 2 * M_PI / 360.0;
      q_cmd.left_arm[i] = toolkit::kJointDirectionFlag[i+7] ? motion_positions[i+7] * 2 * M_PI / 360.0
                                                          : -motion_positions[i+7] * 2 * M_PI / 360.0;
    }

    // count++;
    // if (count == 1000)
    // {
    //   spdlog::info("当前关节0的角度值:{},弧度值{}",
    //                motion_positions[0],
    //                motion_positions[0] * 2 * M_PI / 360.0);
    //   count = 0;
    // }

    return controller::JointPositions(q_cmd);
  };

  robot_->runCycleJointMotion(motion_callback);
}

void MainWindow::on_speedFactorSlider_valueChanged(int value)
{
  // 将滑条值转换为速度因子 (滑条范围 2-50 对应速度因子 0.2-5.0)
  double speedFactor = static_cast<double>(value) * 0.1;

  // 设置到 motion_loader
  if (motion_loader_)
  {
    motion_loader_->setSpeedFactor(speedFactor);
  }

  // 更新界面显示
  ui->speedFactorValue->setText(QString("%1x").arg(speedFactor, 0, 'f', 1));

  // spdlog::info("速度因子已设置为：{}", speedFactor);
}

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
MainWindow::MainWindow(QWidget *parent)
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

  // QStringList items;
  // items << "左轮" << "右轮";
  // ui->comboBox->addItems(items);

  QStringList hand_items1;
  hand_items1 << "左手" << "右手";
  ui->comboBox_hand_target1->addItems(hand_items1);

  QStringList hand_items2;
  hand_items2 << "关节1" << "关节2" << "关节3" << "关节4" << "关节5" << "关节6" << "关节7";
  ui->comboBox_hand_target2->addItems(hand_items2);

  connect(status_update_timer_.get(), &QTimer::timeout, this, &MainWindow::updateUI);
  status_update_timer_->start(200); // 每 200ms 更新一次（5Hz）

  csv_saver_timer_ = std::make_unique<QTimer>(this);
  csv_saver_timer_->setTimerType(Qt::PreciseTimer);
  connect(csv_saver_timer_.get(), &QTimer::timeout, this, &MainWindow::writeCsvSaver);

  ui->spinBox_hand_dq->setValue(100);
  ui->spinBox_hand_tau->setValue(100);

  initStatusTable();
  // 在构造函数或启动前预留空间
  motion_positions_.resize(toolkit::BODY_JOINTS_NUM);
  motion_velocities_.resize(toolkit::BODY_JOINTS_NUM);
  robot_ = controller::createController();
}

MainWindow::~MainWindow()
{
  robot_->emergencyStop();
  delete ui;
}

void MainWindow::initStatusTable()
{
  auto init_status_table = [&](QTableWidget *statusTable, double count,QStringList& headers)
  {
    if (!statusTable)
      return;

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
      QTableWidgetItem *item = new QTableWidgetItem(QString::number(i));
      statusTable->setVerticalHeaderItem(i, item);
    }

    // 初始化所有单元格
    for (int row = 0; row < count; ++row)
    {
      for (int col = 0; col < headers.size(); ++col)
      {
        QTableWidgetItem *item = new QTableWidgetItem("");
        statusTable->setItem(row, col, item);
        item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
      }
    }
  };

  // 设置表头标签
  QStringList headers;
  headers << "关节位置" << "关节速度" << "电机力矩" << "传感器力矩" << "状态字" << "错误字";

  QStringList headers2;
  headers2 << "关节位置" << "期望位置" << "位置偏差" << "关节速度" << "电机力矩" << "状态字";

  init_status_table(ui->statusTable_right, 7, headers);
  init_status_table(ui->statusTable_left, 7, headers);
  init_status_table(ui->statusTable_waist, 4, headers2);
}

void MainWindow::updateStatusTable()
{
  auto state = robot_->getRobotState();

  enum class ArmSelect
  {
    kLeft,
    kRight
  };

  auto update_status_table = [&](QTableWidget *statusTable, ArmSelect arm)
  {
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

      statusTable->item(i, 5)->setText(QString("0x%1").arg(current_arm.error_code.at(i), 4, 16, QChar('0')));

      // statusTable->item(i, 5)->setText(QString::number(current_arm.mode_of_operation.at(i)));
    }
  };

  update_status_table(ui->statusTable_right, ArmSelect::kRight);
  update_status_table(ui->statusTable_left, ArmSelect::kLeft);

  for (auto i = 0; i < 4; ++i)
  {
    ui->statusTable_waist->item(i, 0)->setText(QString::number(state.folded_waist.q.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 1)->setText(QString::number(state.folded_waist.q_d.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 2)->setText(QString::number(state.folded_waist.q_e.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 3)->setText(QString::number(state.folded_waist.dq.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 4)->setText(QString::number(state.folded_waist.tau_J.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 5)->setText(
        QString("0x%1").arg(state.folded_waist.status_word.at(i), 4, 16, QChar('0')));
  }
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
  double target = static_cast<double>(ui->waist_target_slider->value()) / 1000.0;
  ui->waist_target_show->setText(QString::number(target, 'f', 3));

  const double left_wheel_target = static_cast<double>(ui->left_wheel_target_slider->value()) / 100.0;
  const double right_wheel_target = static_cast<double>(ui->right_wheel_target_slider->value()) / 100.0;
  ui->left_wheel_target_show->setText(QString::number(left_wheel_target, 'f', 3));
  ui->right_wheel_target_show->setText(QString::number(right_wheel_target, 'f', 3));

  ui->hand_target_show->setText(QString::number(ui->hand_target_slider->value()));
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

void MainWindow::on_btn_send_waist_target_clicked()
{
  double target = static_cast<double>(ui->waist_target_slider->value()) / 1000.0;
  robot_->setFoldedWaistTarget(target);
}

void MainWindow::on_btn_f1_clicked()
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
      "../file/"
      "left_right_hand_wave20251124.txt"; // 修改为实际文件路径

  // 创建并启动 MotionDataLoader
  motion_loader_ = std::make_unique<toolkit::MotionDataLoader>();

  // 设置回调函数，用于从 motion_loader 获取数据
  motion_loader_->setMotionCallback([this](double timestamp) -> std::vector<double>
                                    {
    std::vector<double> positions;
    std::vector<double> velocities;
    double ts;
    if (motion_loader_->getLatestJointPositions(positions, velocities, ts))
    {
      return positions;
    }
    return std::vector<double>(toolkit::BODY_JOINTS_NUM, 0.0); });

  // 启动离线模式（从文件加载）
  motion_loader_->startOffline(motion_file_path_);

  // 等待数据准备好
  while (!motion_loader_->isDataReady() && motion_loader_->isRunning())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto motion_callback = [this](const controller::RobotState &s,
                                controller::Duration time) -> controller::JointPositions
  {
    controller::ControlCommand q_cmd;
    static int count = 0;
    // 从 motion_loader 获取计算出的数据
    // std::vector<double> motion_positions;
    // std::vector<double> motion_velocities;
    double motion_timestamp;
    if (motion_loader_ && motion_loader_->isDataReady())
    {
      motion_loader_->getLatestJointPositions(motion_positions_, motion_velocities_, motion_timestamp);
    }

    for (int i = 0; i < 7; ++i)
    {
      q_cmd.right_arm[i] = toolkit::kJointDirectionFlag[i] ? motion_positions_[i] * 2 * M_PI / 360.0
                                                           : -motion_positions_[i] * 2 * M_PI / 360.0;
      q_cmd.left_arm[i] = toolkit::kJointDirectionFlag[i + 7] ? motion_positions_[i + 7] * 2 * M_PI / 360.0
                                                              : -motion_positions_[i + 7] * 2 * M_PI / 360.0;
      // auto arc1 = toolkit::kJointDirectionFlag[i] ? motion_positions[i] * 2 * M_PI / 360.0
      //                                             : -motion_positions[i] * 2 * M_PI / 360.0;

      // count++;
      // if (count % 1000 == 0)
      // {
      //   spdlog::info("当前左臂关节0的弧度值:{},弧度值{}", q_cmd.left_arm[i]);
        
      // }
    }

    return controller::JointPositions(q_cmd);
  };

  robot_->runCycleJointMotion(motion_callback);
}

void MainWindow::initCsvSaver()
{
  csv_saver_ = std::make_unique<toolkit::CsvSaver<kCsvDataCount>>("/home/root/csp_log.csv");

  // 添加表头
  std::array<std::string, kCsvDataCount> headers;
  headers[0] = "time";
  for (int i = 0; i < 7; ++i)
  {
    headers[1 + i] = "q_d" + std::to_string(i);       // 左臂关节目标位置
    headers[8 + i] = "q" + std::to_string(i);         // 左臂关节实际位置
    headers[15 + i] = "dq_d" + std::to_string(i);     // 左臂关节目标速度
    headers[22 + i] = "dq" + std::to_string(i);       // 左臂关节实际速度
    headers[29 + i] = "t_motor" + std::to_string(i);  // 左臂关节电机反馈力矩
    headers[36 + i] = "t_sensor" + std::to_string(i); // 左臂关节传感器反馈力矩
  }
  csv_saver_->writeHeaders(headers); // 写入表头

  // csv_saver_timer_->start(100);// 100ms
}

void MainWindow::writeCsvSaver()
{
  auto state = robot_->getRobotState();
  std::array<double, kCsvDataCount> data;
  data[0] = state.time.toSec();

  // 原始数据
  //  std::copy_n(state.left_arm.q_d.begin(), 7, data.begin()+1);
  //  std::copy_n(state.left_arm.q.begin(), 7, data.begin()+8);
  //  std::copy_n(state.left_arm.dq_d.begin(), 7, data.begin() + 15);
  //  std::copy_n(state.left_arm.dq.begin(), 7, data.begin() + 22);
  //  std::copy_n(state.left_arm.tau_J.begin(), 7, data.begin() + 29);
  //  std::copy_n(state.left_arm.tau_Js.begin(), 7, data.begin() + 36);

  // 保留3位小数
  auto round_to_3_decimals = [](double value)
  {
    return std::round(value * 1000.0) / 1000.0;
  };
  // 对每个关节的数据进行处理并保留三位小数
  for (int i = 0; i < 7; ++i)
  {
    data[1 + i] = round_to_3_decimals(state.left_arm.q_d[i]);     // 左臂关节目标位置
    data[8 + i] = round_to_3_decimals(state.left_arm.q[i]);       // 左臂关节实际位置
    data[15 + i] = round_to_3_decimals(state.left_arm.dq_d[i]);   // 左臂关节目标速度
    data[22 + i] = round_to_3_decimals(state.left_arm.dq[i]);     // 左臂关节实际速度
    data[29 + i] = round_to_3_decimals(state.left_arm.tau_J[i]);  // 左臂关节电机反馈力矩
    data[36 + i] = round_to_3_decimals(state.left_arm.tau_Js[i]); // 左臂关节传感器反馈力矩
  }

  csv_saver_->writeRow(data);
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
  auto torque_callback = [](const controller::RobotState &s,
                            controller::Duration time) -> controller::Torques
  {
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

void MainWindow::on_btn_f5_clicked()
{
  robot_->setZeroForceDrag(ui->btn_f5->isChecked());
}

void MainWindow::on_btn_f6_clicked()
{
  controller::JointImpedanceParameter p;

  // ==================== 刚度参数 (Stiffness, Nm/rad) ====================
  // 近端关节(1-3)设定较高刚度以支撑载荷，末端(5-7)设定较低刚度以保持柔顺
  p.left_arm_stiffness = {100.0, 100.0, 80.0, 80.0, 40.0, 20.0, 10.0};
  p.right_arm_stiffness = {100.0, 100.0, 80.0, 80.0, 40.0, 20.0, 10.0};

  // ==================== 阻尼参数 (Damping, Nm·s/rad) ====================
  // 根据 damping = 2 * 0.7 * sqrt(K * I) 估算。
  // 假设近端惯量较大，阻尼相应调高；末端惯量小，阻尼调低以防关节发硬。
  p.left_arm_damping = {30.0, 30.0, 20.0, 20.0, 10.0, 5.0, 2.0};
  p.right_arm_damping = {30.0, 30.0, 20.0, 20.0, 10.0, 5.0, 2.0};

  robot_->runJointImpedance(p);
}

void MainWindow::on_btn_f7_clicked()
{
  controller::CartesianImpedanceParameter p;

  // 稍微降低刚度，提高旋转项的相对阻尼
  // v_x, v_y, v_z, w_x, w_y, w_z
  p.left_arm_stiffness = {150.0, 150.0, 150.0, 10.0, 10.0, 10.0};
  p.right_arm_stiffness = {150.0, 150.0, 150.0, 10.0, 10.0, 10.0};

  // 阻尼调整：遵循 D = 2 * zeta * sqrt(K)，这里假设有效质量为常数进行估算
  // 旋转项 (40->10) 显著降低，因为旋转震荡往往是抖动源
  p.left_arm_damping = {25.0, 25.0, 25.0, 2.0, 2.0, 2.0};
  p.right_arm_damping = {25.0, 25.0, 25.0, 2.0, 2.0, 2.0};

  robot_->runCartesianImpedance(p);
}

void MainWindow::on_btn_f8_clicked()
{
  robot_->stopCurrentMisiion();
  csv_saver_timer_->stop();

  // 停止 motion_loader
  if (motion_loader_)
  {
    motion_loader_->stop();
    motion_loader_.reset();
  }
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

  spdlog::info("速度因子已设置为：{}", speedFactor);
}



void MainWindow::on_btn_send_wheel_target_clicked()
{
  printf("on_btn_send_wheel_target_clicked\n");
  const auto left_vel = static_cast<double>(ui->left_wheel_target_slider->value()) / 100.0;
  const auto right_vel = static_cast<double>(ui->right_wheel_target_slider->value()) / 100.0;
  robot_->setWheelTarget(left_vel, right_vel);
}

void MainWindow::on_btn_stop_wheel_clicked() {

  robot_->setWheelTarget(0.0,0.0);
  ui->left_wheel_target_slider->setValue(0);
  ui->right_wheel_target_slider->setValue(0);
}

void MainWindow::on_btn_send_hand_target_clicked()
{

  controller::HandSelect hand;

  auto t1 = ui->comboBox_hand_target1->currentIndex();
  auto t2 = ui->comboBox_hand_target2->currentIndex();
  auto t3 = static_cast<double>(ui->hand_target_slider->value());
  auto t4 = static_cast<double>(ui->spinBox_hand_dq->value());
  auto t5 = static_cast<double>(ui->spinBox_hand_tau->value());

  if (t1 == 0)
  {
    hand = controller::HandSelect::kLeftHand;
  }
  else if (t1 == 1)
  {
    hand = controller::HandSelect::kRightHand;
  }

  hand_target_.q_d[t2] = t3;
  hand_target_.dq_d[t2] = t4;
  hand_target_.tau_collision[t2] = t5;

  robot_->setHandTarget(hand, hand_target_);

}

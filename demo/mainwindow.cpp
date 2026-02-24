#include "mainwindow.h"

#include <algorithm>
#include <cmath>

#include <QDebug>
#include <QElapsedTimer>
#include <QHeaderView>
#include <QTableWidget>

#include "./ui_mainwindow.h"

// mainwindow.cpp
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), status_update_timer_(std::make_unique<QTimer>(this))
{
  ui->setupUi(this);

  // 设置QLabel属性
  ui->icon->setMinimumSize(150, 50);
  ui->icon->setScaledContents(true);
  QPixmap pix(":/images/icon.png");
  if (!pix.isNull())
  {
    ui->icon->setPixmap(pix);
  }

  connect(status_update_timer_.get(), &QTimer::timeout, this, &MainWindow::updateUI);
  status_update_timer_->start(200);  // 每200ms更新一次（5Hz）

  csv_saver_timer_ = std::make_unique<QTimer>(this);
  csv_saver_timer_->setTimerType(Qt::PreciseTimer);
  connect(csv_saver_timer_.get(), &QTimer::timeout, this, &MainWindow::writeCsvSaver);

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
  auto init_status_table = [&](QTableWidget* statusTable,double count) {
    if (!statusTable) return;

    // 设置表头标签
    QStringList headers;
    headers << "关节位置" << "关节速度" << "电机力矩" << "传感器力矩" << "状态字" << "运行模式";

    // 设置表格行列数
    statusTable->setRowCount(count);
    statusTable->setColumnCount(headers.size());
    statusTable->setHorizontalHeaderLabels(headers);

    // 设置表头行为适应内容
    statusTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    statusTable->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    statusTable->setStyleSheet("QTableWidget { font-size: 9pt; }");

    // 初始化行标题（关节ID）
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

  init_status_table(ui->statusTable_right,7);
  init_status_table(ui->statusTable_left,7);
  init_status_table(ui->statusTable_waist,4);
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

  for (auto i = 0; i < 4; ++i)
  {
    ui->statusTable_waist->item(i, 0)->setText(QString::number(state.folded_waist.q.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 1)->setText(QString::number(state.folded_waist.dq.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 2)->setText(QString::number(state.folded_waist.tau_J.at(i), 'f', 3));

    ui->statusTable_waist->item(i, 4)->setText(QString("0x%1").arg(state.folded_waist.status_word.at(i), 4, 16, QChar('0')));

    ui->statusTable_waist->item(i, 5)->setText(QString::number(state.folded_waist.mode_of_operation.at(i)));
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
      ui->current_mode->setText("CSP使能");
      break;

    case controller::CurrentMode::kCST:
      ui->current_mode->setText("CST使能");
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
  double target =  static_cast<double>(ui->waist_target_slider->value())/ 1000.0;
  ui->waist_target_show->setText(QString::number(target, 'f', 3));

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
  double target =  static_cast<double>(ui->waist_target_slider->value())/ 1000.0;
  robot_->setFoldedWaistTarget(target);
}

void MainWindow::on_btn_f1_clicked()
{
  initCsvSaver();

  struct MotionRange { double min_q; double max_q; };
  const std::map<int, MotionRange> active_config = {
    {0, {-0.5, 0.5}}, // 左臂关节1
    {2, {-0.0, 1.0}}, // 左臂关节3
    {4, {-0.0, 1.0}}, // 左臂关节5
    {5, {-0.4, 0.4}}, // 左臂关节6
    {6, {-0.5, 0.5}}, // 左臂关节7
    {7, {-0.5, 0.5}}, // 右臂关节1
    {9, {-0.0, 1.0}}, // 右臂关节3
    {11, {-0.0, 1.0}}, // 右臂关节5
    {12, {-0.4, 0.4}}, // 右臂关节6
    {13, {-0.5, 0.5}}  // 右臂关节7
  };

  auto motion_callback = [active_config](const controller::RobotState& s,
                            controller::Duration time) -> controller::JointPositions {
    const double t = time.toSec();
    const double t_prep = 2.0;  // 设定 2 秒时间从 0 运动到 q_min
    const double T = 4.0;       // 周期 4s
    const double omega = 2.0 * M_PI / T;
    
    controller::ControlCommand q_cmd;
    // q_cmd.fill(0.0);

    for (int i = 0; i < 7; ++i) {
      if (active_config.count(i)) {
        double q_min = active_config.at(i).min_q;
        double q_max = active_config.at(i).max_q;
        double amp = (q_max - q_min) / 2.0;
        double mid = (q_max + q_min) / 2.0;

        if (t < t_prep) {
          // === 阶段 A: 从 0 平滑过渡到 q_min ===
          // 使用 1 - cos 曲线实现从速度 0 起步，到速度 0 结束
          // 公式：q = (target/2) * (1 - cos(pi * t / t_prep))
          double prep_omega = M_PI / t_prep;
          q_cmd.left_arm[i] = (q_min / 2.0) * (1.0 - std::cos(prep_omega * t));
        } 
        else {
          // === 阶段 B: 正式的周期运动 ===
          // 修正时间偏移量，确保从 t = t_prep 时刻开始接续
          double t_cycle = t - t_prep; 
          q_cmd.left_arm[i] = mid - amp * std::cos(omega * t_cycle);
        }
      } else {
        q_cmd.left_arm[i] = 0.0;
      }
    }

    return controller::JointPositions(q_cmd);
  };

  robot_->runCycleJointMotion(motion_callback);
}

void MainWindow::initCsvSaver()
{
  csv_saver_ = std::make_unique<toolkit::CsvSaver<kCsvDataCount>>("/home/root/csp_log.csv");
  csv_saver_timer_->start(10);
}

void MainWindow::writeCsvSaver()
{
  // printf("writeCsvSaver\n");
  auto state = robot_->getRobotState();
  std::array<double, kCsvDataCount> data;
  std::copy_n(state.left_arm.q.begin(), 7, data.begin());
  std::copy_n(state.right_arm.q.begin(), 7, data.begin() + 7);
  std::copy_n(state.left_arm.dq.begin(), 7, data.begin() + 14);
  std::copy_n(state.right_arm.dq.begin(), 7, data.begin() + 21);
  std::copy_n(state.left_arm.tau_J.begin(), 7, data.begin() + 28);
  std::copy_n(state.right_arm.tau_J.begin(), 7, data.begin() + 35);
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
  auto torque_callback = [](const controller::RobotState& s,
                            controller::Duration time) -> controller::Torques {
    // 1. 获取当前总时间（秒）
    const double t = time.toSec();
    std::array<double, 14> tau_cmd{0.0};

    // 2. 正弦运动参数
    constexpr double A     = 5.0;  // 振幅 5.0 N*m
    constexpr double T     = 4.0;  // 周期 4.0 s
    constexpr double omega = 2.0 * M_PI / T;

    double q1_target = A * std::sin(omega * t);

    tau_cmd[0] = q1_target;

    // 调试打印：每隔约 500ms 打印一次
    if (time.toMSec() % 500 == 0)
    {
      printf("Time: %.2f s, Target Q1: %.3f\n", t, q1_target);
    }

    // 5. 使用你定义的 std::array 构造函数返回
    return controller::Torques(tau_cmd);
  };

  // 开启运动
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
  p.left_arm_stiffness  = {100.0, 100.0, 80.0, 80.0, 40.0, 20.0, 10.0};
  p.right_arm_stiffness = {100.0, 100.0, 80.0, 80.0, 40.0, 20.0, 10.0};

  // ==================== 阻尼参数 (Damping, Nm·s/rad) ====================
  // 根据 damping = 2 * 0.7 * sqrt(K * I) 估算。
  // 假设近端惯量较大，阻尼相应调高；末端惯量小，阻尼调低以防关节发硬。
  p.left_arm_damping  = {30.0, 30.0, 20.0, 20.0, 10.0, 5.0, 2.0};
  p.right_arm_damping = {30.0, 30.0, 20.0, 20.0, 10.0, 5.0, 2.0};

  robot_->runJointImpedance(p);
}

void MainWindow::on_btn_f7_clicked()
{
  controller::CartesianImpedanceParameter p;

  // 稍微降低刚度，提高旋转项的相对阻尼
  // v_x, v_y, v_z, w_x, w_y, w_z
  p.left_arm_stiffness  = {150.0, 150.0, 150.0, 10.0, 10.0, 10.0};
  p.right_arm_stiffness = {150.0, 150.0, 150.0, 10.0, 10.0, 10.0};

  // 阻尼调整：遵循 D = 2 * zeta * sqrt(K)，这里假设有效质量为常数进行估算
  // 旋转项 (40->10) 显著降低，因为旋转震荡往往是抖动源
  p.left_arm_damping  = {25.0, 25.0, 25.0, 2.0, 2.0, 2.0};
  p.right_arm_damping = {25.0, 25.0, 25.0, 2.0, 2.0, 2.0};

  robot_->runCartesianImpedance(p);
}

void MainWindow::on_btn_f8_clicked()
{
  robot_->stopCurrentMisiion();
}
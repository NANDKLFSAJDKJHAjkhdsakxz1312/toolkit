#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <cstdint>
#include <memory>
#include <string>

#include "csv_saver.hpp"
#include "motion_data_loader.h"
#include "ZdlController/controller_api.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
  class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

protected:
  void initStatusTable();
  void updateStatusTable();
  void updateUI();
  void initCsvSaver();
  void writeCsvSaver();

private:
  Ui::MainWindow *ui;
  std::unique_ptr<controller::ControllerInterface> robot_;
  std::unique_ptr<QTimer> status_update_timer_;

  static constexpr int kCsvDataCount = 43;
  std::unique_ptr<toolkit::CsvSaver<kCsvDataCount>> csv_saver_;
  std::unique_ptr<QTimer> csv_saver_timer_;

  // MotionDataLoader 成员
  std::unique_ptr<toolkit::MotionDataLoader> motion_loader_;
  std::string motion_file_path_;

  std::vector<double> motion_positions_;
  std::vector<double> motion_velocities_;

  controller::HandCommand hand_target_{255.0};

private slots:
  void on_btn_e_stop_clicked();
  void on_btn_f1_clicked();
  void on_btn_f2_clicked();
  void on_btn_forward_pressed();
  void on_btn_forward_released();
  void on_btn_reverse_pressed();
  void on_btn_reverse_released();
  void on_btn_f4_clicked();
  void on_btn_f5_clicked();
  void on_btn_f6_clicked();
  void on_btn_f7_clicked();
  void on_btn_f8_clicked();
  void on_btn_f9_clicked();

  void on_btn_s1_clicked();
  void on_btn_s2_clicked();
  void on_btn_s3_clicked();
  void on_btn_s4_clicked();
  void on_btn_s5_clicked();

  void on_btn_send_waist_target_clicked();
  void on_btn_send_wheel_target_clicked();
  void on_btn_stop_wheel_clicked();

  void on_btn_send_hand_target_clicked();

  void on_speedFactorSlider_valueChanged(int value);
};
#endif // MAINWINDOW_H

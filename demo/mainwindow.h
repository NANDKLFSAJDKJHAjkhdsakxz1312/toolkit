#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <cstdint>
#include <memory>

#include "csv_saver.hpp"

#include "controller_api.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 protected:
  void initStatusTable();
  void updateStatusTable();
  void updateUI();
  void initCsvSaver();
  void writeCsvSaver();

 private:
  Ui::MainWindow* ui;
  std::unique_ptr<controller::ControllerInterface> robot_;
  std::unique_ptr<QTimer> status_update_timer_;

  static constexpr int kCsvDataCount = 22;
  std::unique_ptr<toolkit::CsvSaver<kCsvDataCount>> csv_saver_;
  std::unique_ptr<QTimer> csv_saver_timer_;

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
  void on_btn_s1_clicked();
  void on_btn_s2_clicked();
  void on_btn_s3_clicked();
  void on_btn_s4_clicked();
  void on_btn_s5_clicked();

  void on_btn_send_waist_target_clicked();

};
#endif  // MAINWINDOW_H

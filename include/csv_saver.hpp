#pragma once

#include <fstream>
#include <string>
#include <array>
#include <cstddef>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <charconv> // C++17 核心库

namespace toolkit {

template<std::size_t N>
class CsvSaver {
 public:
  explicit CsvSaver(const std::string& file_path = "data.csv") 
      : stop_(false) {
      file_.open(file_path);
      if (file_.is_open()) {
          worker_ = std::thread(&CsvSaver::processQueue, this);
      }
      std::cout << std::fixed << std::setprecision(1);
  }
  
  ~CsvSaver() {
      {
          std::unique_lock<std::mutex> lock(mtx_);
          stop_ = true;
      }
      cv_.notify_all(); 
      if (worker_.joinable()) {
          worker_.join();
      }
      if (file_.is_open()) {
          file_.close();
      }
  }
  
  void writeHeaders(const std::array<std::string, N>& headers) {
      if (!file_.is_open()) return;
      for (std::size_t i = 0; i < N; ++i) {
          file_ << headers[i] << (i == N - 1 ? "" : ",");
      }
      file_ << "\n";
      file_.flush();
  }

  void writeRow(const std::array<double, N>& data) {
      {
          std::lock_guard<std::mutex> lock(mtx_);
          if (stop_) return;
          data_queue_.push(data);
      }
      cv_.notify_one(); 
  }

 private:
  void processQueue() {
      std::queue<std::array<double, N>> local_queue;
      
      auto last_report_time = std::chrono::steady_clock::now();
      long long total_duration_us = 0;
      long long total_rows_written = 0;

      // 预分配一个缓冲区，避免在循环中重复申请内存
      // double 转字符最多约 24 字节，N个double + 逗号，分配足够的空间
      std::vector<char> line_buffer(N * 32); 

      while (true) {
          {
              std::unique_lock<std::mutex> lock(mtx_);
              cv_.wait(lock, [this] { return !data_queue_.empty() || stop_; });
              
              if (stop_ && data_queue_.empty()) return;
              std::swap(local_queue, data_queue_);
          }
          
          auto start_write = std::chrono::steady_clock::now();
          std::size_t batch_size = local_queue.size();

          while (!local_queue.empty()) {
              const auto& row = local_queue.front();
              
              for (std::size_t i = 0; i < N; ++i) {
                  // 使用 std::to_chars 进行极速转换
                  auto [ptr, ec] = std::to_chars(line_buffer.data(),
                                                 line_buffer.data() + line_buffer.size(),
                                                 row[i]
                                                //  std::chars_format::fixed, // 核心修改：指定固定格式
                                                //  6  // 核心修改：指定精度
                                                 );                     
                  if (ec == std::errc()) {
                      // 写入转换后的字符段
                      file_.write(line_buffer.data(), ptr - line_buffer.data());
                  }
                  
                  if (i != N - 1) {
                      file_.put(',');
                  }
              }
              file_.put('\n');
              local_queue.pop();
          }
          // 在 1000Hz 下，不要频繁 flush。只有当没有积压数据时才 flush
          file_.flush();

          auto end_write = std::chrono::steady_clock::now();
          total_duration_us += std::chrono::duration_cast<std::chrono::microseconds>(end_write - start_write).count();
          total_rows_written += batch_size;

          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::seconds>(now - last_report_time).count() >= 1) {
              if (total_rows_written > 0) {
                  double avg_time = static_cast<double>(total_duration_us) / total_rows_written;
                  std::cout << "[CsvSaver Info] Avg write time: " << avg_time 
                            << " us/row (Total rows this sec: " << total_rows_written << ")" << "\n";
              }
              total_duration_us = 0;
              total_rows_written = 0;
              last_report_time = now;
          }
      }
  }

  std::ofstream file_;
  std::queue<std::array<double, N>> data_queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread worker_;
  std::atomic<bool> stop_;
};

}  // namespace toolkit
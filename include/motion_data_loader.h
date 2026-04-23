#pragma once

#include <pthread.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace toolkit {

// =============================================================
// 基本常量定义
// =============================================================
constexpr int ARM_JOINT_NUM = 7;
constexpr int BODY_JOINTS_NUM = 16;  // 身体关节总数

// =============================================================
// 数据结构定义
// =============================================================
// 关节方向标志：true 表示保持原方向，false 表示需要取反
// 对应 13 个关节：[0,4,10,11] 为正方向，其余为负方向
constexpr std::array<bool, 14> kJointDirectionFlag = {false,
                                                      false,
                                                      false,
                                                      true,
                                                      true,
                                                      false,
                                                      true,

                                                      true,
                                                      false,
                                                      false,
                                                      false,
                                                      true,
                                                      false,
                                                      true};  

// 单关节系数
struct JointSplineCoefficients
{
  Eigen::VectorXd cubicCoeffs;                // [a, b, c, d]
  Eigen::Matrix<double, 6, 1> quinticCoeffs;  // [a, b, c, d, e, f]
  int jointIndex = -1;

  JointSplineCoefficients()
  {
    cubicCoeffs.setZero();
    quinticCoeffs.setZero();
  }
};

// 一帧（或一小段）所有关节的系数
struct BatchSplineCoefficients
{
  std::vector<JointSplineCoefficients> jointCoeffs;
  long batchIndex = 0;
  int batchSize = 0;
  double startTime = 0.0;
  double endTime = 0.0;
  double quinticDuration = 0.0;
  std::vector<double> segmentDurations;

  BatchSplineCoefficients(int numJoints = BODY_JOINTS_NUM)
  {
    jointCoeffs.resize(numJoints);
    segmentDurations.reserve(100);
  }

  // 拷贝函数
  void copyFrom(const BatchSplineCoefficients& src)
  {
    batchIndex = src.batchIndex;
    batchSize = src.batchSize;
    startTime = src.startTime;
    endTime = src.endTime;
    quinticDuration = src.quinticDuration;
    segmentDurations = src.segmentDurations;
    if (jointCoeffs.size() == src.jointCoeffs.size())
    {
      jointCoeffs = src.jointCoeffs;
    }
  }
};

// 环形缓冲区（简化版，无锁/定容）
class SplineCoefficientsRingBuffer
{
 private:
  static const size_t BUFFER_SIZE = 32;
  std::vector<BatchSplineCoefficients> buffer;
  std::atomic<size_t> readIndex{0};
  std::atomic<size_t> writeIndex{0};
  std::atomic<bool> m_cancelRequested{false};
  std::mutex mutex;
  std::condition_variable cv_data;
  std::condition_variable cv_space;

 public:
  SplineCoefficientsRingBuffer(int numJoints = BODY_JOINTS_NUM);
  void clear();
  void cancel();
  bool push_rt(BatchSplineCoefficients& coeffs);
  bool try_pop_rt(BatchSplineCoefficients& target);
  bool isEmpty() const;
  bool isFull() const;
};

// =============================================================
// 轨迹数据处理器（生产者 - 消费者模式）
// =============================================================
class MotionDataLoader
{
 public:
  using MotionCallback = std::function<std::vector<double>(double timestamp)>;

  MotionDataLoader();
  ~MotionDataLoader();

  // 启动离线模式（从文件加载）
  void startOffline(const std::string& filePath);

  // 停止所有线程
  void stop();

  // 设置 motion 回调函数
  void setMotionCallback(MotionCallback callback);

  // 获取计算出的关节位置（由消费者线程高频更新）
  bool getLatestJointPositions(std::vector<double>& positions,
                               std::vector<double>& velocities,
                               double& timestamp);

  // 检查数据是否可用
  bool isDataReady() const { return m_isDataReady.load(); }
  bool isRunning() const { return m_isRunning.load(); }

 private:
  // 生产者线程函数
  static void* producerThreadFunc(void* arg);
  // 消费者线程函数
  static void* consumerThreadFunc(void* arg);

  // 实际执行逻辑
  void runProducer();
  void runConsumer();

  // 辅助函数
  void setThreadAffinity(int coreId);
  void setRealtimePriority();

  // 样条插值计算
  void loadOfflineData(const std::string& path);
  Eigen::VectorXd filter(const Eigen::VectorXd& input);
  void offlineCom(const std::atomic<bool>& cancelRequested);
  Eigen::VectorXd pathSegmentInterpolation_cubicSplines(const Eigen::VectorXd& viaPointSeries_position,
                                                        const Eigen::VectorXd& timeStampSeries,
                                                        double startPoint_velocity,
                                                        double endPoint_velocity);
  Eigen::VectorXd calculateQuinticSplineCoefficients(double startPos,
                                                     double endPos,
                                                     double duration,
                                                     const Eigen::Vector4d& prevCubicCoeffs,
                                                     double prevDuration,
                                                     const Eigen::Vector4d& nextCubicCoeffs);

  // 上采样计算
  void upSample();
  void calculatePolynomials(double t, bool forceZeroVelAcc);
  bool loadNextSegment();
  void switchToQuinticBridge();
  void switchToCubicSegment(int segmentIdx);

 private:
  // 数据成员
  std::string m_offlineFilePath;
  SplineCoefficientsRingBuffer* m_ringBuffer = nullptr;
  BatchSplineCoefficients* m_batchCoeffs = nullptr;
  BatchSplineCoefficients m_tempBatchCoeffs;

  // 线程句柄
  pthread_t m_producerThread = 0;
  pthread_t m_consumerThread = 0;

  // 控制标志
  std::atomic<bool> m_isRunning{false};
  std::atomic<bool> m_isDataReady{false};

  // 回调函数
  MotionCallback m_motionCallback;
  std::mutex m_callbackMutex;

  // 输出数据缓存
  std::vector<double> m_jointPosition;
  std::vector<double> m_jointVelocity;
  std::vector<double> m_jointAcceleration;
  double m_t_global = 0.0;

  // 状态结构
  enum SegmentType { CUBIC_SEGMENT, QUINTIC_BRIDGE };
  struct RuntimeState
  {
    double localTime = 0.0;
    double currentDuration = 0.0;
    double lastBatchEndTime = 0.0;
    int currentCubicIndex = 0;
    SegmentType currentType = CUBIC_SEGMENT;
    std::vector<Eigen::Matrix<double, 6, 1>> currentCoeffs;

    RuntimeState() { currentCoeffs.resize(BODY_JOINTS_NUM, Eigen::Matrix<double, 6, 1>::Zero()); }
  } m_state;

  // 离线数据
  std::vector<Eigen::VectorXd> m_filteredDataForAllJoints;
  std::vector<Eigen::VectorXd> m_data;
  Eigen::VectorXd m_timeStamp;
  std::vector<Eigen::VectorXd> m_viaPoints;
  int m_jointNum = 0;
  int m_pointNum = 0;
  int m_BATCH_NUM = 0;
  static const int BATCH_SIZE = 3;

  // 成员变量
  // 用于保存上一个 Batch 最后一个 segment 的系数和时长，实现平滑衔接
  std::vector<Eigen::Vector4d> m_prevBatchLastCubic;  // 每个关节一个 Vector4d
  double m_prevBatchLastDuration = 0.0;
  bool m_isFirstRound = true;

  // 速度因子 (用于调速)
  std::atomic<double> m_speedFactor{1.0};  // 速度范围 0.2-5.0

 public:
  // 速度因子接口
  double getSpeedFactor() const { return m_speedFactor.load(); }
  void setSpeedFactor(double factor) { m_speedFactor.store(factor); }
};

}  // namespace toolkit

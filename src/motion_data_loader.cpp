#include "motion_data_loader.h"

#include <cmath>
#include <sstream>

namespace toolkit {

// =============================================================
// SplineCoefficientsRingBuffer 实现
// =============================================================
SplineCoefficientsRingBuffer::SplineCoefficientsRingBuffer(int numJoints)
{
  buffer.resize(BUFFER_SIZE, BatchSplineCoefficients(numJoints));
}

void SplineCoefficientsRingBuffer::clear()
{
  readIndex.store(0);
  writeIndex.store(0);
  m_cancelRequested.store(false);
}

void SplineCoefficientsRingBuffer::cancel()
{
  m_cancelRequested.store(true);
  cv_data.notify_all();
  cv_space.notify_all();
}


bool SplineCoefficientsRingBuffer::push_rt(BatchSplineCoefficients& coeffs)
{
  std::unique_lock<std::mutex> lock(mutex);

  // 等待空间 (Producer 不是 RT 关键路径，可以等待)
  cv_space.wait(lock, [this] {
    size_t nextWrite = (writeIndex + 1) % BUFFER_SIZE;
    return (nextWrite != readIndex) || m_cancelRequested;
  });

  if (m_cancelRequested) return false;

  // 【关键优化】原本是 copyFrom (深拷贝)，现在改为 swap。
  // 这意味着传入的 coeffs 会被掏空（变成 buffer 里旧的垃圾数据）。
  // 因为 coeffs 是 Producer 的局部临时变量 m_tempBatchCoeffs，
  // 它变成垃圾没关系，下次 Producer 会重新填充它。
  std::swap(buffer[writeIndex], coeffs);

  writeIndex = (writeIndex + 1) % BUFFER_SIZE;

  lock.unlock();
  cv_data.notify_one();
  return true;
}

bool SplineCoefficientsRingBuffer::try_pop_rt(BatchSplineCoefficients& target)
{
  std::unique_lock<std::mutex> lock(mutex);

  if (readIndex == writeIndex) return false;  // 真的空了

  // 极速交换，持有锁的时间极短
  std::swap(target, buffer[readIndex]);

  readIndex = (readIndex + 1) % BUFFER_SIZE;

  // lock 析构自动解锁
  cv_space.notify_one();
  return true;
}

bool SplineCoefficientsRingBuffer::isEmpty() const
{
  return readIndex.load() == writeIndex.load();
}

bool SplineCoefficientsRingBuffer::isFull() const
{
  return ((writeIndex.load() + 1) % BUFFER_SIZE) == readIndex.load();
}

// =============================================================
// MotionDataLoader 实现
// =============================================================
MotionDataLoader::MotionDataLoader()
    : m_batchCoeffs(new BatchSplineCoefficients(BODY_JOINTS_NUM)),
      m_tempBatchCoeffs(BODY_JOINTS_NUM),
      m_ringBuffer(new SplineCoefficientsRingBuffer(BODY_JOINTS_NUM)),
      m_jointPosition(BODY_JOINTS_NUM, 0.0),
      m_jointVelocity(BODY_JOINTS_NUM, 0.0),
      m_jointAcceleration(BODY_JOINTS_NUM, 0.0)
{
}

MotionDataLoader::~MotionDataLoader()
{
  stop();

  if (m_ringBuffer)
  {
    delete m_ringBuffer;
    m_ringBuffer = nullptr;
  }
  if (m_batchCoeffs)
  {
    delete m_batchCoeffs;
    m_batchCoeffs = nullptr;
  }
}

void MotionDataLoader::startOffline(const std::string& filePath)
{
  if (m_isRunning.load())
    return;

  m_offlineFilePath = filePath;
  m_isRunning.store(true);
  m_isDataReady.store(false);

  pthread_create(&m_producerThread, nullptr, &MotionDataLoader::producerThreadFunc, this);
  pthread_create(&m_consumerThread, nullptr, &MotionDataLoader::consumerThreadFunc, this);
}

void MotionDataLoader::stop()
{
  if (!m_isRunning.load())
    return;

  m_isRunning.store(false);

  if (m_ringBuffer)
    m_ringBuffer->cancel();

  if (m_producerThread)
  {
    pthread_join(m_producerThread, nullptr);
    m_producerThread = 0;
  }
  if (m_consumerThread)
  {
    pthread_join(m_consumerThread, nullptr);
    m_consumerThread = 0;
  }
}

void MotionDataLoader::setMotionCallback(MotionCallback callback)
{
  std::lock_guard<std::mutex> lock(m_callbackMutex);
  m_motionCallback = callback;
}

bool MotionDataLoader::getLatestJointPositions(std::vector<double>& positions,
                                               std::vector<double>& velocities, double& timestamp)
{
  if (!m_isDataReady.load())
    return false;

  positions     = m_jointPosition;
  velocities    = m_jointVelocity;
  timestamp     = m_t_global;
  return true;
}

void* MotionDataLoader::producerThreadFunc(void* arg)
{
  static_cast<MotionDataLoader*>(arg)->runProducer();
  return nullptr;
}

void* MotionDataLoader::consumerThreadFunc(void* arg)
{
  static_cast<MotionDataLoader*>(arg)->runConsumer();
  return nullptr;
}

void MotionDataLoader::runProducer()
{
  setRealtimePriority();
  setThreadAffinity(4);
  spdlog::info("MotionDataLoader Producer 线程启动");

  try
  {
    // 1. 【关键】数据只在启动时加载一次到内存
    // 避免在循环中进行文件 IO 和大规模滤波运算，这些是导致末端卡顿的主因
    loadOfflineData(m_offlineFilePath);

    // 内存屏障，确保数据加载完成后再通知消费者
    std::atomic_thread_fence(std::memory_order_release);
    m_isDataReady.store(true);
    spdlog::info("离线数据预加载完成，点数: {}, 线程开始循环生成轨迹", m_pointNum);

    while (m_isRunning.load())
    {
      // 2. 【核心】offlineCom 内部会不断 push_rt
      // 因为 RingBuffer 的 push_rt 内部有 cv_space.wait()
      // 当缓冲区满（BUFFER_SIZE=10）时，生产者会在这里自然阻塞，不占用 CPU
      // 只要消费者取走一个，这里就会立即计算并填充下一个，保证了数据流的绝对连续
      offlineCom(m_isRunning);

      if (!m_isRunning.load()) break;

      // 3. 一轮播放结束后的处理
      spdlog::debug("一轮轨迹系数计算推送完成，立即开始下一轮衔接");
    }
  } 
  catch (const std::exception& e)
  {
    spdlog::error("Producer 核心异常：{}", e.what());
    m_isRunning.store(false);
    m_ringBuffer->cancel(); // 唤醒可能的阻塞
  }

  spdlog::info("Producer 线程退出");
}

void MotionDataLoader::runConsumer()
{
  setRealtimePriority();
  setThreadAffinity(5);
  spdlog::info("MotionDataLoader Consumer 线程启动");

  while (m_isRunning.load() && !m_isDataReady.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (!m_isRunning.load())
    return;

  struct timespec next_period;
  clock_gettime(CLOCK_MONOTONIC, &next_period);
  const long PERIOD_NS = 1000000;  // 1ms

  while (m_isRunning.load())
  {
    upSample();

    next_period.tv_nsec += PERIOD_NS;
    while (next_period.tv_nsec >= 1000000000)
    {
      next_period.tv_nsec -= 1000000000;
      next_period.tv_sec++;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (now.tv_sec > next_period.tv_sec ||
        (now.tv_sec == next_period.tv_sec && now.tv_nsec > next_period.tv_nsec))
    {
      next_period = now;
    }
    else
    {
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
  }

  spdlog::info("Consumer 线程退出");
}

void MotionDataLoader::setRealtimePriority()
{
  struct sched_param param;
  param.sched_priority = 80;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
  {
    spdlog::warn("无法设置实时优先级，请检查权限 (sudo)");
  }
}

void MotionDataLoader::setThreadAffinity(int coreId)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(coreId, &cpuset);
  int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (rc != 0)
  {
    spdlog::warn("绑定核心 {} 失败：{}", coreId, strerror(rc));
  }
  else
  {
    spdlog::info("线程已绑定到 Core {}", coreId);
  }
}

void MotionDataLoader::loadOfflineData(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open())
  {
    throw std::runtime_error("无法打开文件：" + path);
  }

  std::string line;
  int totalColumns = 0;
  std::vector<std::vector<double>> columnBuffers;

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    std::istringstream iss(line);
    std::vector<double> rowBuffer;
    double value;

    while (iss >> value)
    {
      rowBuffer.push_back(value);
    }

    if (columnBuffers.empty())
    {
      totalColumns = rowBuffer.size();
      m_jointNum    = totalColumns - 1;  // 第一列是时间戳
      columnBuffers.resize(totalColumns);
    }

    if (rowBuffer.size() != totalColumns)
    {
      spdlog::warn("列数不匹配，跳过该行");
      continue;
    }

    for (int col = 0; col < totalColumns; ++col)
    {
      columnBuffers[col].push_back(rowBuffer[col]);
    }
  }
  file.close();

  if (columnBuffers.empty())
  {
    throw std::runtime_error("数据容器为空！");
  }

  m_data.reserve(totalColumns);
  for (auto& col : columnBuffers)
  {
    m_data.push_back(Eigen::Map<Eigen::VectorXd>(col.data(), col.size()));
  }

  m_timeStamp = m_data[0];
  m_viaPoints.clear();
  m_viaPoints.insert(m_viaPoints.end(), m_data.begin() + 1, m_data.end());

  // 滤波处理
  m_filteredDataForAllJoints.resize(m_jointNum);
  for (int i = 0; i < m_jointNum; i++)
  {
    m_filteredDataForAllJoints[i] = filter(m_viaPoints[i]);
  }

  m_pointNum  = m_filteredDataForAllJoints[0].size();
  m_BATCH_NUM = m_pointNum / BATCH_SIZE;

  spdlog::info("离线数据加载完毕：pointNum={}, BATCH_NUM={}", m_pointNum, m_BATCH_NUM);
}

Eigen::VectorXd MotionDataLoader::filter(const Eigen::VectorXd& input)
{
  if (input.size() == 0)
    return input;

  const Eigen::VectorXd b = Eigen::Map<const Eigen::VectorXd>(
      std::vector<double>{0.004824343357716, 0.019297373430865, 0.028946060146297, 0.019297373430865,
                          0.004824343357716}
          .data(),
      5);
  const Eigen::VectorXd a = Eigen::Map<const Eigen::VectorXd>(
      std::vector<double>{1.0, -2.369513007182036, 2.313988414415877, -1.054665405878565, 0.187379492368184}
          .data(),
      5);

  const int filter_order = b.size() - 1;
  double initial_val     = input[0];

  Eigen::VectorXd x_history = Eigen::VectorXd::Constant(filter_order + 1, initial_val);
  Eigen::VectorXd y_history = Eigen::VectorXd::Constant(filter_order + 1, initial_val);

  Eigen::VectorXd via_points(input.size());

  for (int i = 0; i < input.size(); ++i)
  {
    x_history.tail(filter_order) = x_history.head(filter_order);
    y_history.tail(filter_order) = y_history.head(filter_order);
    x_history[0]                 = input[i];

    double y = b.dot(x_history) - a.tail(filter_order).dot(y_history.tail(filter_order));
    y /= a[0];

    y_history[0]  = y;
    via_points[i] = y;
  }
  return via_points;
}

void MotionDataLoader::offlineCom(const std::atomic<bool>& isRunning)
{
  if (!isRunning.load())
  {
    spdlog::info("系数计算被取消");
    return;
  }

  const int SYSTEM_JOINT_NUM = BODY_JOINTS_NUM;

  if (m_BATCH_NUM >= 2)
  {
    for (int batchIdx = 0; batchIdx < m_BATCH_NUM; batchIdx++)
    {
      if (!isRunning.load())
        return;

      BatchSplineCoefficients batchCoeffs(SYSTEM_JOINT_NUM);
      batchCoeffs.batchIndex = batchIdx;

      int startIdx = BATCH_SIZE * batchIdx;
      int currentBatchSize = (batchIdx == (m_BATCH_NUM - 1)) ? (m_pointNum - startIdx) : BATCH_SIZE;

      int numCubicCoeffs = (currentBatchSize - 1) * 4;
      for (auto& joint : batchCoeffs.jointCoeffs)
      {
        joint.cubicCoeffs.resize(numCubicCoeffs);
        joint.cubicCoeffs.setZero();
      }

      batchCoeffs.batchSize   = currentBatchSize;
      batchCoeffs.startTime   = m_timeStamp(startIdx);
      batchCoeffs.endTime     = m_timeStamp(startIdx + currentBatchSize - 1);
      batchCoeffs.quinticDuration = (batchIdx > 0) ? (m_timeStamp(startIdx) - m_timeStamp(startIdx - 1)) : 0.0;

      batchCoeffs.segmentDurations.reserve(currentBatchSize - 1);
      for (int k = 0; k < currentBatchSize - 1; ++k)
      {
        double dt = m_timeStamp(startIdx + k + 1) - m_timeStamp(startIdx + k);
        batchCoeffs.segmentDurations.push_back(dt);
      }

      for (int jointIdx = 0; jointIdx < SYSTEM_JOINT_NUM; jointIdx++)
      {
        if (!isRunning.load())
        {
          spdlog::info("计算被取消，退出循环 (Batch {})", batchIdx);
          return;
        }

        if (jointIdx >= m_jointNum)
        {
          batchCoeffs.jointCoeffs[jointIdx].cubicCoeffs = Eigen::VectorXd::Zero((currentBatchSize - 1) * 4);
          batchCoeffs.jointCoeffs[jointIdx].quinticCoeffs = Eigen::VectorXd::Zero(6);
          batchCoeffs.jointCoeffs[jointIdx].jointIndex = jointIdx;
          continue;
        }

        double velocityOfLeftStart, velocityOfRightEnd;

        if (batchIdx == (m_BATCH_NUM - 1))
        {
          velocityOfLeftStart = (m_filteredDataForAllJoints[jointIdx](startIdx + 1) -
                                 m_filteredDataForAllJoints[jointIdx](startIdx - 1)) /
                                (m_timeStamp(startIdx + 1) - m_timeStamp(startIdx - 1));
          velocityOfRightEnd = 0;

          auto coeffs = pathSegmentInterpolation_cubicSplines(
              m_filteredDataForAllJoints[jointIdx].tail(currentBatchSize),
              m_timeStamp.tail(currentBatchSize), velocityOfLeftStart, velocityOfRightEnd);

          batchCoeffs.jointCoeffs[jointIdx].cubicCoeffs = coeffs;
        }
        else
        {
          if (batchIdx == 0)
            velocityOfLeftStart = 0;
          else
            velocityOfLeftStart = (m_filteredDataForAllJoints[jointIdx](startIdx + 1) -
                                   m_filteredDataForAllJoints[jointIdx](startIdx - 1)) /
                                  (m_timeStamp(startIdx + 1) - m_timeStamp(startIdx - 1));

          velocityOfRightEnd = (m_filteredDataForAllJoints[jointIdx](startIdx + BATCH_SIZE) -
                                m_filteredDataForAllJoints[jointIdx](startIdx + BATCH_SIZE - 2)) /
                               (m_timeStamp(startIdx + BATCH_SIZE) - m_timeStamp(startIdx + BATCH_SIZE - 2));

          auto coeffs = pathSegmentInterpolation_cubicSplines(
              m_filteredDataForAllJoints[jointIdx].segment(startIdx, BATCH_SIZE),
              m_timeStamp.segment(startIdx, BATCH_SIZE), velocityOfLeftStart, velocityOfRightEnd);

          batchCoeffs.jointCoeffs[jointIdx].cubicCoeffs = coeffs;
        }

        batchCoeffs.jointCoeffs[jointIdx].jointIndex = jointIdx;

        if (batchIdx > 0)
        {
          double pos_start = m_filteredDataForAllJoints[jointIdx](startIdx - 1);
          double pos_end   = m_filteredDataForAllJoints[jointIdx](startIdx);
          double gap_dur   = batchCoeffs.quinticDuration;

          double prev_last_seg_duration = m_timeStamp(startIdx - 1) - m_timeStamp(startIdx - 2);

          Eigen::Vector4d prevCubicCoeffs = Eigen::Vector4d::Zero();
          // 简化处理，实际应用中需要从之前的 batch 获取

          Eigen::Vector4d nextCubicCoeffs = batchCoeffs.jointCoeffs[jointIdx].cubicCoeffs.head(4);

          Eigen::VectorXd quintic = calculateQuinticSplineCoefficients(
              pos_start, pos_end, gap_dur, prevCubicCoeffs, prev_last_seg_duration, nextCubicCoeffs);

          batchCoeffs.jointCoeffs[jointIdx].quinticCoeffs = quintic;
        }
      }

      m_ringBuffer->push_rt(batchCoeffs);

      if (!isRunning.load())
      {
        spdlog::info("检测到取消信号 (Batch {})", batchIdx);
        return;
      }
    }
  }
}

Eigen::VectorXd MotionDataLoader::pathSegmentInterpolation_cubicSplines(
    const Eigen::VectorXd& viaPointSeries_position, const Eigen::VectorXd& timeStampSeries,
    double startPoint_velocity, double endPoint_velocity)
{
  const int pointNum              = viaPointSeries_position.size();
  const int cubicSplineFittingNum = pointNum - 1;
  const int systemSize            = 4 * cubicSplineFittingNum;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(systemSize, systemSize);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(systemSize);

  A(0, 3) = 1.0;
  b(0)    = viaPointSeries_position[0];

  A(1, 2) = 1.0;
  b(1)    = startPoint_velocity;

  for (int i = 1; i < cubicSplineFittingNum; ++i)
  {
    const int splineIdx = i - 1;
    const double deltaT = timeStampSeries[i] - timeStampSeries[i - 1];

    const int rowPos = 2 + 4 * (i - 1);
    A.block(rowPos, 4 * splineIdx, 1, 4) << std::pow(deltaT, 3), std::pow(deltaT, 2), deltaT, 1.0;
    b[rowPos] = viaPointSeries_position[i];

    A.block(rowPos + 1, 4 * (splineIdx + 1), 1, 4) << 0.0, 0.0, 0.0, 1.0;
    b[rowPos + 1] = viaPointSeries_position[i];

    const int rowVel = rowPos + 2;
    A.block(rowVel, 4 * splineIdx, 1, 3) << 3 * std::pow(deltaT, 2), 2 * deltaT, 1.0;
    A.block(rowVel, 4 * (splineIdx + 1), 1, 3) << -0.0, -0.0, -1.0;

    const int rowAcc = rowPos + 3;
    A.block(rowAcc, 4 * splineIdx, 1, 2) << 6 * deltaT, 2.0;
    A.block(rowAcc, 4 * (splineIdx + 1), 1, 2) << -0.0, -2.0;
  }

  const int lastSplineIdx  = cubicSplineFittingNum - 1;
  const double finalDeltaT = timeStampSeries[pointNum - 1] - timeStampSeries[pointNum - 2];

  const int lastPosRow = systemSize - 2;
  A.block(lastPosRow, 4 * lastSplineIdx, 1, 4) << std::pow(finalDeltaT, 3), std::pow(finalDeltaT, 2), finalDeltaT, 1.0;
  b[lastPosRow] = viaPointSeries_position[pointNum - 1];

  A.block(lastPosRow + 1, 4 * lastSplineIdx, 1, 3) << 3 * std::pow(finalDeltaT, 2), 2 * finalDeltaT, 1.0;
  b[lastPosRow + 1] = endPoint_velocity;

  Eigen::VectorXd coefficients = A.inverse() * b;

  if ((A * coefficients - b).norm() > 1e-6)
  {
    throw std::runtime_error("Cubic spline solution accuracy not acceptable");
  }
  return coefficients;
}

Eigen::VectorXd MotionDataLoader::calculateQuinticSplineCoefficients(
    double startPos, double endPos, double duration, const Eigen::Vector4d& prevCubicCoeffs,
    double prevDuration, const Eigen::Vector4d& nextCubicCoeffs)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(6);

  double v_start = 3 * prevCubicCoeffs(0) * std::pow(prevDuration, 2) + 2 * prevCubicCoeffs(1) * prevDuration +
                   prevCubicCoeffs(2);
  double a_start = 6 * prevCubicCoeffs(0) * prevDuration + 2 * prevCubicCoeffs(1);

  double v_end = nextCubicCoeffs(2);
  double a_end = 2 * nextCubicCoeffs(1);

  A(0, 5) = 1.0;
  b(0)    = startPos;

  A.row(1) << std::pow(duration, 5), std::pow(duration, 4), std::pow(duration, 3), std::pow(duration, 2), duration, 1.0;
  b(1) = endPos;

  A(2, 4) = 1.0;
  b(2)    = v_start;

  A.row(3) << 5 * std::pow(duration, 4), 4 * std::pow(duration, 3), 3 * std::pow(duration, 2), 2 * duration, 1.0, 0.0;
  b(3) = v_end;

  A(4, 3) = 2.0;
  b(4)    = a_start;

  A.row(5) << 20 * std::pow(duration, 3), 12 * std::pow(duration, 2), 6 * duration, 2.0, 0.0, 0.0;
  b(5) = a_end;

  return A.colPivHouseholderQr().solve(b);
}

void MotionDataLoader::upSample()
{
  static bool wasStarved = false;

  if (!m_ringBuffer)
    return;

  // 1. 时间步进 (这部分逻辑不变)
  constexpr double kTimeStep = 0.001;  // 消费者是 1000Hz
  double currentSpeedFactor  = m_speedFactor.load();
  double dt                  = kTimeStep * currentSpeedFactor;
  m_state.localTime += dt;
  m_t_global += dt;

  while (m_state.localTime >= m_state.currentDuration)
  {
    m_state.localTime -= m_state.currentDuration;

    bool hasNext = loadNextSegment();
    if (!hasNext)
    {
      m_state.localTime = m_state.currentDuration;

      if (!wasStarved)
      {
        spdlog::error("[Consumer] Buffer EMPTY! Holding last position.");
        wasStarved = true;
      }

      goto CALCULATE_OUTPUT;
    }
  }

  if (wasStarved)
  {
    spdlog::info("[Consumer] Buffer Recovered. Resuming motion.");
    wasStarved = false;
  }

CALCULATE_OUTPUT:
  calculatePolynomials(m_state.localTime, wasStarved);
}

void MotionDataLoader::calculatePolynomials(double t, bool forceZeroVelAcc)
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  Eigen::Matrix<double, 6, 1> posBasis;
  posBasis << t5, t4, t3, t2, t, 1.0;

  Eigen::Matrix<double, 6, 1> velBasis;
  Eigen::Matrix<double, 6, 1> accBasis;

  if (!forceZeroVelAcc)
  {
    velBasis << 5 * t4, 4 * t3, 3 * t2, 2 * t, 1.0, 0.0;
    accBasis << 20 * t3, 12 * t2, 6 * t, 2.0, 0.0, 0.0;
  }

  for (int i = 0; i < BODY_JOINTS_NUM; i++)
  {
    const auto& coeffs = m_state.currentCoeffs[i];

    m_jointPosition[i] = posBasis.dot(coeffs);

    if (forceZeroVelAcc)
    {
      m_jointVelocity[i]     = 0.0;
      m_jointAcceleration[i] = 0.0;
    }
    else
    {
      m_jointVelocity[i]     = velBasis.dot(coeffs);
      m_jointAcceleration[i] = accBasis.dot(coeffs);
    }
  }
}

bool MotionDataLoader::loadNextSegment()
{
  if (!m_batchCoeffs->segmentDurations.empty())
  {
    if (m_state.currentType == QUINTIC_BRIDGE)
    {
      switchToCubicSegment(0);
      return true;
    }

    int totalSegments = static_cast<int>(m_batchCoeffs->segmentDurations.size());
    if (m_state.currentCubicIndex < totalSegments - 1)
    {
      switchToCubicSegment(m_state.currentCubicIndex + 1);
      return true;
    }
  }

  if (m_ringBuffer->try_pop_rt(m_tempBatchCoeffs))
  {
    std::swap(*m_batchCoeffs, m_tempBatchCoeffs);

    m_state.currentCubicIndex = -1;

    if (m_batchCoeffs->jointCoeffs.size() != BODY_JOINTS_NUM)
    {
      spdlog::error("Joint size mismatch in new batch!");
      return false;
    }

    if (m_batchCoeffs->batchIndex > 0 && m_batchCoeffs->quinticDuration > 1e-6)
    {
      switchToQuinticBridge();
    }
    else
    {
      switchToCubicSegment(0);
    }
    return true;
  }

  return false;
}

void MotionDataLoader::switchToQuinticBridge()
{
  m_state.currentType = QUINTIC_BRIDGE;
  m_state.localTime   = 0.0;

  double duration = m_batchCoeffs->quinticDuration;
  if (duration <= 1e-6)
    duration = 0.01;
  m_state.currentDuration = duration;

  for (int i = 0; i < BODY_JOINTS_NUM; i++)
  {
    m_state.currentCoeffs[i] = m_batchCoeffs->jointCoeffs[i].quinticCoeffs;
  }
}

void MotionDataLoader::switchToCubicSegment(int segmentIdx)
{
  m_state.currentType       = CUBIC_SEGMENT;
  m_state.currentCubicIndex = segmentIdx;
  m_state.localTime         = 0.0;

  if (segmentIdx < (int)m_batchCoeffs->segmentDurations.size())
  {
    m_state.currentDuration = m_batchCoeffs->segmentDurations[segmentIdx];
  }
  else
  {
    spdlog::error("Segment index out of range!");
    m_state.currentDuration = 0.01;
  }

  for (int i = 0; i < BODY_JOINTS_NUM; i++)
  {
    const auto& srcVec = m_batchCoeffs->jointCoeffs[i].cubicCoeffs;
    int startIndex     = 4 * segmentIdx;

    if (startIndex + 4 > (int)srcVec.size())
    {
      m_state.currentCoeffs[i].setZero();
      continue;
    }

    Eigen::Vector4d cubic = srcVec.segment<4>(startIndex);
    m_state.currentCoeffs[i] << 0.0, 0.0, cubic(0), cubic(1), cubic(2), cubic(3);
  }
}

}  // namespace toolkit

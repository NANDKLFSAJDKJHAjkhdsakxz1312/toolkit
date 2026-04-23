#pragma once

#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef ZDL_IGH_LIB
#define EC_API __declspec(dllexport)
#else
#define EC_API __declspec(dllimport)
#endif
#else
#define EC_API __attribute__((visibility("default")))
#endif

#include <functional>
#include <memory>
#include <string>
#include <vector>

// #include "config.h"
#include "ec_sdo_api.h"

struct MasterImpl;

class EC_API ecmaster
{
public:
  explicit ecmaster(const std::string &config_path);
  ~ecmaster();

  bool initialize();
  bool start();
  void stop();
  bool isRunning() const;
  uint64_t getCycleCount() const;
  size_t getProcessDataSize() const;

  using CycleCallback = std::function<void(uint8_t *, size_t, uint64_t)>;
  using ConfigCallback = std::function<void()>;

  void set_config_callback(ConfigCallback callback);
  void set_receive_callback(ConfigCallback callback);
  void set_cycle_callback(ConfigCallback callback);
  void set_test_callback(ConfigCallback callback);

  struct PdoEntryInfo
  {
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length; // 动态模式必填；静态模式如果填0会自动推导
  };

  // static std::unique_ptr<MasterConfig> loadConfig(const std::string &filepath);

  // ========================================================================
  //                          模式 A: 动态映射 (Dynamic)
  // ========================================================================
  // 描述：完全重写从站的 PDO 配置 (Sync Managers)。
  // 限制：不能与静态映射函数混用。
  // ========================================================================
  template <typename T>
  bool register_dynamic_pdo(T *&addr,
                            uint16_t slave_position,
                            const PdoEntryInfo &entry_info,
                            bool is_RX_sl, // true=slave_RxPDO(Input), false=slave_TxPDO(Output)
                            uint16_t slave_alias = 0)
  {
    PdoEntryInfo info_copy = entry_info;
    if (info_copy.bit_length == 0)
      info_copy.bit_length = sizeof(T) * 8;
    // 内部实现：Mode=Dynamic
    return register_pdo_internal((void **)&addr, slave_position, info_copy, is_RX_sl, slave_alias, true);
  }

  // ========================================================================
  //                          模式 B: 静态映射 (Static)
  // ========================================================================
  // 描述：使用 XML/ESI 默认配置，仅注册数据指针。
  // 限制：不能与动态映射函数混用。
  // ========================================================================
  template <typename T>
  bool register_static_pdo(T *&addr,
                           uint16_t slave_position,
                           const PdoEntryInfo &entry_info,
                           uint16_t slave_alias = 0)
  {
    PdoEntryInfo info_copy = entry_info;
    if (info_copy.bit_length == 0)
      info_copy.bit_length = sizeof(T) * 8;
    // 内部实现：Mode=Static, is_RX_sl 参数在静态模式下被忽略(设为true占位)
    return register_pdo_internal((void **)&addr, slave_position, info_copy, true, slave_alias, false);
  }

  // SDO 绑定 (通用)
  template <typename T>
  bool bind_sdo(EcApi::Sdo<T> &sdo, uint16_t slave_position, uint16_t index, uint8_t subindex)
  {
    return bind_sdo_internal(static_cast<EcApi::SdoBase &>(sdo), slave_position, index, subindex, sizeof(T));
  }

  // 状态获取
  struct MasterState
  {
    unsigned int slaves_responding;
    unsigned int al_states;
    bool link_up;
  };
  struct DomainState
  {
    unsigned int working_counter;
    unsigned int wc_state;
    unsigned int redundancy_active;
  };
  struct SlaveState
  {
    bool online;
    bool operational;
    uint8_t al_state;
  };

  MasterState get_master_state() const;
  DomainState get_domain_state() const;
  SlaveState get_slave_state(size_t slave_index) const;
  std::vector<SlaveState> get_all_slave_states() const;

  void updateDiagnostics();

private:
  std::unique_ptr<MasterImpl> m_pImpl;

  // 统一内部接口
  bool register_pdo_internal(void **addr_ptr,
                             uint16_t slave_pos,
                             const PdoEntryInfo &info,
                             bool is_RX_sl,
                             uint16_t alias,
                             bool is_dynamic_mode);

  bool bind_sdo_internal(EcApi::SdoBase &sdo, uint16_t slave_pos, uint16_t index, uint8_t subindex, size_t size);
};
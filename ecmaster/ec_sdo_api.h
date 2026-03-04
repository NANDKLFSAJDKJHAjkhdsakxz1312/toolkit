#ifndef EC_SDO_API_H
#define EC_SDO_API_H

#include <cstdint>
#include <memory> 
#include <type_traits>

#if defined(_WIN32) || defined(__CYGWIN__)
  #ifdef BUILDING_MY_LIB
    #define EC_API __declspec(dllexport)
  #else
    #define EC_API __declspec(dllimport)
  #endif
#else
  #define EC_API __attribute__((visibility("default")))
#endif
// =========================================================

struct ec_sdo_request;

namespace EcApi {

struct SdoImpl;

class EC_API SdoBase {
 public:
  SdoBase();
  virtual ~SdoBase();

  SdoBase(const SdoBase&) = default;
  SdoBase& operator=(const SdoBase&) = default;
  SdoBase(SdoBase&&) = default;
  SdoBase& operator=(SdoBase&&) = default;

  void request_read(); 
  bool is_busy() const;
  bool is_success() const;
  bool is_error() const;

  void bind_handle(struct ec_sdo_request *req);
  void update(); 

 protected:
  void write_internal(const void *data, size_t size);
  void read_internal(void *data, size_t size) const;

 private:
  std::shared_ptr<SdoImpl> impl_;
};

template <typename T>
class Sdo : public SdoBase {
 public:
  static_assert(sizeof(T) <= 8, "Sdo type size too large (>8 bytes)");

  Sdo() = default;

  void write(T val) { write_internal(&val, sizeof(T)); }

  T read() const
  {
    T val = {};
    read_internal(&val, sizeof(T));
    return val;
  }
  
  bool try_get_value(T &out_val) const
  {
    if (is_success())
    {
      out_val = read();
      return true;
    }
    return false;
  }
};

}  // namespace EcApi

#endif  // EC_SDO_API_H
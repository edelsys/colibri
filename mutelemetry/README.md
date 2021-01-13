#  MuTelemetry

## Desription
ULog-based telemetry framework which allows streaming of ULog messages through network (with mavlink2 protocol) and/or saving them to file.

## Build
Refer to Colibri library build instructions.

## How to use
1. Allow mutelemetry in muconfig.toml. If *log_directory_path* is not set, then log will be saved in current directory.
```cpp
[mutelemetry]
    with_local_log = true
    with_network = true
    log_directory_path = "/var/log/telemetry_logs"
```
2. Try to initialize mutelemetry and check if initialization has succeeded.
```cpp
  MuTelemetry::init();
  MuTelemetry &mt = MuTelemetry::getInstance();
  if (!mt.is_enabled()) {
      // smth went wrong
	  return -1;
  }
  // Your code goes here
```
3.  You may want to register some information and parameters, for example:
```cpp
  mt.register_info("company", "EDEL LLC");
  mt.register_info("sys_name", "RBPi4");
  mt.register_info("replay", mt.get_logname());
  mt.register_param("int32_t param1", 123);
```
4. Provide definitions of data types (formats) and ways to serialize them according to their formats. 
```cpp
  mt.register_data_format(DataType0::name(), DataType0::fields());
```
   *DataType0* is defined as following:
```cpp
struct DataType0 {
  uint64_t __timestamp = 0;
  double a = 1111.2222;
  unsigned int b = 3;
  short int c = -4;
  
  static inline const std::string& name() {
	  static const std::string name_(#DataType0);
	  return name_;
  }

  static inline const std::string& fields() {
	  static const std::string fields_("double a;uint32_t b;int16_t c;");
	  return fields_;
  }
  
  std::vector<uint8_t> serialize() {
	  std::vector<uint8_t> serialized(sizeof(DataType0));
	  size_t len = 0;
	  std::memcpy(&serialized[0], &__timestamp, sizeof(__timestamp));
	  len += sizeof(__timestamp);
	  std::memcpy(&serialized[len], &a, sizeof(a));
	  len += sizeof(a);
	  std::memcpy(&serialized[len], &b, sizeof(b));
	  len += sizeof(b);
	  std::memcpy(&serialized[len], &c, sizeof(c));
	  return serialized;
  }
};
```
5. There are several ways to provide serialization. This example is based on *Serializable* interface. Refer to mutelemetry_test.cc for more approaches.
```cpp
class DataType0Serializable : public Serializable, public DataType0 {
 public:
  vector<uint8_t> serialize() override {
	  return DataType0::serialize();
  }
};
```
6. Send your data to mutelemetry with *store_data()* method.
```cpp
bool result = MuTelemetry::getInstance().store_data(
	  shared_ptr<Serializable>{new DataType0Serializable},
	  DataType0::name(), "dataType0_instance1");
```

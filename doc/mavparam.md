## Mavlink parameter protocol API

Mavlink [parameter protocol](https://mavlink.io/en/services/parameter.html) works in 'muroute' out of the box.
To be able to use it each component should inherit [BaseComponent](/muroute/include/muroute/componentbus.h) class with
desired component Id and be managed by component bus (i.e. registered within component bus). This Id is transparently mapped to mavlink 
component Id. See [Components](components.md) for more information about component bus and components.

```cpp
RouteSystem::RouteSystem()
    : BaseComponent::BaseComponent(MAV_COMP_ID_ONBOARD_COMPUTER)
```

'BaseComponent' provides a convenient way to work with parameters.
To add parameters in a component setParameterValue() should be used.

```cpp
  setParameterValue("PARAM1", 2.0f);
```

There are overloaded functions for different types of parameters.

```cpp
  bool setParameterValue(const std::string &, const std::string &, int);
  bool setParameterValue(const std::string &, double);
  bool setParameterValue(const std::string &, float);
  bool setParameterValue(const std::string &, uint64_t);
  bool setParameterValue(const std::string &, int64_t);
  bool setParameterValue(const std::string &, uint32_t);
  bool setParameterValue(const std::string &, int32_t);
  bool setParameterValue(const std::string &, uint16_t);
  bool setParameterValue(const std::string &, int16_t);
  bool setParameterValue(const std::string &, uint8_t);
  bool setParameterValue(const std::string &, int8_t);
```

By a protocol limitation the name of parameters shouldn't go beyond 16 bytes.
Colibri suppports [extended parameters protocol](https://mavlink.io/en/services/parameter_ext.html) which can work with string types.

Components should override 'updateParameterValue(const std::string &, const ParamUnion &)' method to be able to accept and update parameters.

This is how it's done for example in routing subsystem (see [subsytem.cc](/muroute/src/subsystem.cc)):

```cpp
bool RouteSystem::updateParameterImpl(const string &param_id,
                                      const MavParams::ParamUnion &value) {
  bool result = false;
  if (param_id.compare(PARAMETER_FORWARDING) == 0) {
    setForwarding(value.param_uint8);
    result = true;
  }
  return result;
}
```

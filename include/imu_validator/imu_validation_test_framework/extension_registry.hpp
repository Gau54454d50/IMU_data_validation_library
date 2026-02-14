#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <string>
#include <unordered_map>

namespace imu_validator::test_framework
{
template<typename InterfaceT, typename... Args>
class ExtensionRegistry
{
public:
  using Factory = std::function<std::unique_ptr<InterfaceT>(Args...)>;

  void register_factory(const std::string & extension_name, Factory factory)
  {
    factories_[extension_name] = std::move(factory);
  }

  std::unique_ptr<InterfaceT> create(const std::string & extension_name, Args... args) const
  {
    const auto it = factories_.find(extension_name);
    if (it == factories_.end()) {
      throw std::invalid_argument("Unknown extension: " + extension_name);
    }
    return it->second(std::forward<Args>(args)...);
  }

private:
  std::unordered_map<std::string, Factory> factories_;
};
}  // namespace imu_validator::test_framework

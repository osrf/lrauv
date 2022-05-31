#include "lrauv_system_tests/Publisher.hh"

#include <thread>

#include "lrauv_system_tests/Util.hh"

namespace lrauv_system_tests
{

bool WaitForConnections(
  const gz::transport::Node::Publisher &_publisher,
  const std::chrono::nanoseconds _timeout)
{
  Timeout timeout{_timeout};
  while (!_publisher.HasConnections() && !timeout)
  {
    using namespace std::literals::chrono_literals;
    constexpr std::chrono::nanoseconds minSleepTime = 100ms;
    std::this_thread::sleep_for(
        std::min(timeout.TimeRemaining(), minSleepTime));
  }
  return _publisher.HasConnections();
}

}

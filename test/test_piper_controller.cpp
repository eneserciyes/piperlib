#include "controller_base.h"
#include "spdlog/spdlog.h"

int main() {
  try {
    ControllerConfig controller_config;
    spdlog::info("Controller config: {}", controller_config.interface_name);
    PiperController joint_controller(controller_config);

    if (!joint_controller.start()) {
      spdlog::error("Failed to start controller");
      return 1;
    }

    spdlog::info("Controller started. Commanding various joint states...");

    sleep_ms(1000);

    joint_controller.setTarget({0.2, 0.2, -0.2, 0.3, -0.2, 0.5}, 0.0f);
    sleep_ms(3000);

    joint_controller.setTarget({-0.1, 0.3, -0.1, -0.2, -0.4, -0.3}, 0.5f);
    sleep_ms(3000);

    joint_controller.setTarget({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0f);
    sleep_ms(3000);

    joint_controller.setTarget({0.1, 0.1, -0.2, 0.1, -0.1, 0.2}, 0.5f);
    sleep_ms(3000);

    joint_controller.setTarget({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0f);
    sleep_ms(3000);

    spdlog::info("Test completed, stopping controller");
    joint_controller.stop();

  } catch (const std::exception &e) {
    spdlog::error("Error: {}", e.what());
    return 1;
  }

  spdlog::info("Test finished successfully");
  return 0;
}
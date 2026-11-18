#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <typeinfo>

int main() {
  try {
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
        "controller_interface", "controller_interface::ControllerInterface");
    const auto classes = loader.getDeclaredClasses();
    const std::string vanilla = "cartesian_impedance_controller/CartesianImpedanceController";
    const std::string force = "force_impedance_controller/ForceImpedanceController";
    if (std::find(classes.begin(), classes.end(), vanilla) == classes.end() ||
        std::find(classes.begin(), classes.end(), force) == classes.end())
      throw std::runtime_error("Both plugin names must be discoverable in the ament index");
    auto pose_controller = loader.createSharedInstance(vanilla);
    auto force_controller = loader.createSharedInstance(force);
    if (typeid(*pose_controller) == typeid(*force_controller))
      throw std::runtime_error("Force and vanilla plugins must instantiate different classes");
    std::cout << "PASS both plugins discovered, loaded and distinct in one process\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}

# Automatically generated by: ros2nix --distro jazzy --flake --license Apache-2.0

# Copyright 2025 None
# Distributed under the terms of the Apache-2.0 license

{ lib, buildRosPackage, ament-cmake, ament-lint-common, cartesian-controller-base, cartesian-force-controller, cartesian-motion-controller, controller-interface, hardware-interface, pluginlib, rclcpp }:
buildRosPackage rec {
  pname = "ros-jazzy-cartesian-compliance-controller";
  version = "0.0.0";

  src = ./.;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  checkInputs = [ ament-lint-common ];
  propagatedBuildInputs = [ cartesian-controller-base cartesian-force-controller cartesian-motion-controller controller-interface hardware-interface pluginlib rclcpp ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "Control your robot through Cartesian target poses and target wrenches";
    license = with lib.licenses; [ bsd3 ];
  };
}

{ stdenv
, lib
, cmake
, fetchFromGitHub
, DynamixelSDK
, dynamixel-workbench
, yaml-cpp
, spdlog
}:

stdenv.mkDerivation rec {
  pname = "wx_armor";
  version = "1.0.0";

  src = ./.;

  nativeBuildInputs = [ cmake ];

  buildInputs = [
    DynamixelSDK
    dynamixel-workbench
    yaml-cpp
    spdlog
  ];

  meta = with lib; {
    description = "Onboard WidowX driver based on Dynamixel SDK";
    homepage = "https://github.com/HorizonRoboticsInternal/interbotix_x";
    license = licenses.asl20;
    maintainers = with maintainers; [ breakds ];
  };

}

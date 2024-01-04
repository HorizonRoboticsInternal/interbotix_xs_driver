{ stdenv
, lib
, cmake
, fetchFromGitHub
, DynamixelSDK
, dynamixel-workbench
, yaml-cpp
, spdlog
, nlohmann_json
, drogon
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
    nlohmann_json
    drogon
  ];

  postInstall = ''
    mkdir -p $out/etc/wx_armor
    ln -s $src/configs/wx250s_motor_config.yaml $out/etc/wx_armor
  '';

  meta = with lib; {
    description = "Onboard WidowX driver based on Dynamixel SDK";
    homepage = "https://github.com/HorizonRoboticsInternal/interbotix_x";
    license = licenses.asl20;
    maintainers = with maintainers; [ breakds ];
  };

}

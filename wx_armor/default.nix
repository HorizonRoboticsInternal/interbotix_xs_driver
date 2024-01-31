{ stdenv
, lib
, makeWrapper
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

  nativeBuildInputs = [ cmake makeWrapper ];

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
    makeWrapper $out/bin/wx_armor $out/bin/debug_wx_armor \
        --set WX_ARMOR_MOTOR_CONFIG $out/etc/wx_armor/wx250s_motor_config.yaml
  '';

  meta = with lib; {
    description = "Onboard WidowX driver based on Dynamixel SDK";
    homepage = "https://github.com/HorizonRoboticsInternal/interbotix_x";
    license = licenses.asl20;
    maintainers = with maintainers; [ breakds ];
  };
}

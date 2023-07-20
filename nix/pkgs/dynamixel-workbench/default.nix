{ stdenv
, lib
, cmake
, fetchFromGitHub
, DynamixelSDK
, validatePkgConfig
}:

stdenv.mkDerivation rec {
  pname = "dynamixel-workbench";
  version = "2022.08.22";

  src = fetchFromGitHub {
    owner = "Interbotix";
    repo = pname;
    rev = "88d336bf2552395b704f39044d379ccf8005d03d";
    hash = "sha256-eZMd7xTjS3gQ1W09WjRxWY5AYp4/PZvAWprJ2HpDgv4=";
  };

  nativeBuildInputs = [ cmake validatePkgConfig ];

  buildInputs = [
    DynamixelSDK
  ];

  meta = with lib; {
    description = "ROS packages for Dynamixel controllers, msgs, single_manager, toolbox, tutorials";
    homepage = "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/";
    license = licenses.asl20;
    maintainers = with maintainers; [ breakds ];
  };
}

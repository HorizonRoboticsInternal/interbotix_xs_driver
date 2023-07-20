{ stdenv
, lib
, cmake
, fetchFromGitHub
}:

stdenv.mkDerivation rec {
  pname = "DynamixelSDK";
  version = "2022.08.22";

  src = fetchFromGitHub {
    owner = "Interbotix";
    repo = pname;
    rev = "733533dbfd6620564883207a20172e00f4a3aad4";
    hash = "sha256-4iDDb4REMiGQw4uD2AkGdbosoTwXx/mL0SaiDlToay4=";
  };

  nativeBuildInputs = [ cmake ];

  meta = with lib; {
    description = "Trossen Robotics fork of the ROBOTIS Dynamixel SDK (Protocol1.0/2.0)";
    homepage = "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/";
    license = licenses.asl20;
    maintainers = with maintainers; [ breakds ];
  };
}

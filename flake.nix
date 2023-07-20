{
  description = "Provides low-level interfaces to easily control an Interbotix X-Series Robot";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/23.05";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils, ... }: {
    overlays.dev = final: prev: {
      DynamixelSDK = final.callPackage ./nix/pkgs/DynamixelSDK {};
      dynamixel-workbench = final.callPackage ./nix/pkgs/dynamixel-workbench {};
    };
    overlays.default = nixpkgs.lib.composeManyExtensions [
      self.overlays.dev
    ];
  } // utils.lib.eachSystem [
    "x86_64-linux" "aarch64-linux"
  ] (system:
    let pkgs-dev = import nixpkgs {
          inherit system;
          overlays = [ self.overlays.dev ];
        };

        pkgs = import nixpkgs {
          inherit system;
          overlays = [ self.overlays.default ];
        };

    in {
      devShells.default = pkgs-dev.mkShell rec {
        name = "interbotix";

        packages = with pkgs-dev; [
          gcc
          cmake
          cmakeCurses
          pkgconfig
          
          yaml-cpp
          DynamixelSDK
          dynamixel-workbench
        ];

        shellHook = ''
          export PS1="$(echo -e '\uf1c0') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
        '';        
      };

      packages = {
        inherit (pkgs) DynamixelSDK dynamixel-workbench;
      };
    });
}

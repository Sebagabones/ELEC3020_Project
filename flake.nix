{
  description = "A Python development environment";

  inputs = { nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05"; };

  outputs = { self, nixpkgs }: {
    devShells.x86_64-linux.default =
      nixpkgs.legacyPackages.x86_64-linux.mkShell {
        packages = with nixpkgs.legacyPackages.x86_64-linux; [
          (python312.withPackages (ps: with ps; [ pip setuptools ]))
          platformio
          git
          libffi
          pkg-config
          libusb-compat-0_1
          openssl
          avrdude
          esptool
        ];

        # Environment variables for your Python shell
        shellHook = ''
          export PYTHONPATH=$(pwd)
          echo "Welcome to your Python development shell!"
        '';
      };
  };
}

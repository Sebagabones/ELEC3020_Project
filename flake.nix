{
  description = "embedded systems devshell";

  inputs = { nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable"; };

  outputs = { self, nixpkgs }:
    let pkgs = import nixpkgs { system = "x86_64-linux"; };
    in {
      devShells.x86_64-linux.default = pkgs.mkShell rec {
        nativeBuildInputs = with pkgs; [
          (python312.withPackages (ps:
            with ps; [
              pip
              setuptools
              catppuccin
              pygments
              pandas
              numpy
              matplotlib
              pyqt6
              pygments-markdown-lexer
            ]))
          platformio
          git
          libffi
          pkg-config
          libusb-compat-0_1
          openssl
          avrdude
          esptool
          (texlive.combine {
            inherit (texlive)
              scheme-small latexmk luatex graphics enumitem type1cm printlen
              multirow cm-super svg transparent svg-inkscape circuitikz capt-of
              dvisvgm dvipng wrapfig amsmath ulem hyperref fontspec listings
              xcolor koma-script lstfiracode fvextra upquote lineno tcolorbox
              sectsty minted catppuccinpalette pdfcol caption latex-graphics-dev
              booktabs;
          })
          inkscape
        ];

        # Environment variables for your Python shell
        shellHook = ''
          export PYTHONPATH=$(pwd)
        '';
      };
    };
}

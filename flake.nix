{
  description = "";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
  };
  outputs =
    inputs:
    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      forEachSupportedSystem =
        f:
        inputs.nixpkgs.lib.genAttrs supportedSystems (
          system:
          f {
            pkgs = import inputs.nixpkgs { inherit system; };
            inherit system;
          }
        );
    in
    {
      devShells = forEachSupportedSystem (
        { pkgs, system }:
        let 
          #pythonEnv = {
          #  pkgs.python313Packages.astropy
          #  pkgs.python313Packages.numpy
          #};
        in
        {
          default = pkgs.mkShell {
            buildInputs = [
              pkgs.python313
              pkgs.python313Packages.astropy
              pkgs.gnumake
              pkgs.gcc
              pkgs.pkg-config
              pkgs.virtualenv
            ];
          };
        }
      );
    };
}

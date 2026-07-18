{
  description = "Voxel Game Engine development environment";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { nixpkgs, ... }:
    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
          nativeLibraries = with pkgs; [
            alsa-lib
            udev
            wayland
            libxkbcommon
            vulkan-loader
            libx11
            libxcursor
            libxi
            libxrandr
          ];
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              pkg-config
              rustup
            ];

            buildInputs = nativeLibraries;

            # Winit and wgpu load these libraries at runtime rather than link
            # them directly. NixOS GPU drivers are exposed through /run.
            LD_LIBRARY_PATH = "${pkgs.lib.makeLibraryPath nativeLibraries}:/run/opengl-driver/lib";
          };
        });
    };
}

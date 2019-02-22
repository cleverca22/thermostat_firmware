{ nixpkgs ? builtins.fetchTarball "https://github.com/nixos/nixpkgs/archive/be445a9074f.tar.gz" }:
let
  pkgs = import nixpkgs {};
in pkgs.pkgsCross.avr.stdenv.mkDerivation {
  name = "firmware";
  src = ./.;
  installPhase = ''
    mkdir $out
    cp avr.bin $out/
  '';
}

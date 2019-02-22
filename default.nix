let
  pkgs = import <nixpkgs> {};
in pkgs.pkgsCross.avr.stdenv.mkDerivation {
  name = "firmware";
  src = ./.;
  installPhase = ''
    mkdir $out
    cp avr.bin $out/
  '';
}

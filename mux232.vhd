library IEEE;
use IEEE.std_logic_1164.all;

entity mux232 is
  port(	
    d0, d1 : in std_logic_vector(31 downto 0);
    s      : in std_logic;
    y	   : out std_logic_vector(31 downto 0)
  );
end mux232;

architecture behavior of mux232 is
begin
  y <= d0 when s = '0' else d1;
end behavior;
library IEEE;
use IEEE.std_logic_1164.all;

entity mux332 is
  port(	
    d0, d1, d2 : in std_logic_vector(31 downto 0);
    s          : in std_logic_vector(1 downto 0);
    y	       : out std_logic_vector(31 downto 0)
  );
end mux332;

architecture behavior of mux332 is
begin
    with s select
        y <= d0 when "00",
            d1 when "01",
            d2 when "10",
            (others => '-') when others;
end architecture;
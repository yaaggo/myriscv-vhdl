library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity adder32 is
    port(
        a : in  std_logic_vector(31 downto 0); -- entrada a: primeiro operando de 32 bits
        b : in  std_logic_vector(31 downto 0); -- entrada b: segundo operando de 32 bits
        s : out std_logic_vector(31 downto 0)  -- saida s: resultado da soma de a + b
    );
end adder32;

architecture behavior of adder32 is
begin
    -- atribui a 's' o resultado da soma de 'a' e 'b'
    -- e necessario converter os tipos para realizar a operacao aritmetica
    s <= std_logic_vector(unsigned(a) + unsigned(b));
end behavior;
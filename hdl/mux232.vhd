library IEEE;
use IEEE.std_logic_1164.all;

entity mux232 is
    port( 
        -- entradas: duas fontes de dados de 32 bits
        d0, d1 : in  std_logic_vector(31 downto 0);
        
        -- entrada: sinal de selecao de 1 bit
        s      : in  std_logic;
        
        -- saida: saida de 32 bits selecionada
        y      : out std_logic_vector(31 downto 0)
    );
end mux232;

architecture behavior of mux232 is
begin
    -- atribui 'd0' a 'y' se 's' for '0', senao atribui 'd1'
    y <= d0 when s = '0' else d1;
end behavior;
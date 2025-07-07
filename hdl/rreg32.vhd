library ieee;
use ieee.std_logic_1164.all;

entity rreg32 is
    port(
        clk : in  std_logic;                       -- entrada: sinal de clock do sistema
        rst : in  std_logic;                       -- entrada: sinal de reset para inicializacao
        d   : in  std_logic_vector(31 downto 0);   -- entrada: dado de 32 bits a ser armazenado
        q   : out std_logic_vector(31 downto 0)    -- saida: dado de 32 bits atualmente armazenado
    );
end rreg32;

architecture behavior of rreg32 is
begin
    -- define um processo sensivel aos sinais de clock e reset
    process(clk, rst)
    begin
        -- checa se o sinal de reset esta ativo (reset assincrono)
        if (rst = '1') then
            -- se o reset estiver ativo, a saida 'q' eh zerada
            q <= (others => '0');
        -- senao, na borda de subida do clock...
        elsif (rising_edge(clk)) then
            -- a saida 'q' recebe o valor da entrada 'd'
            q <= d;
        end if;
    end process;
end behavior;
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity alu is
    port(
        -- entradas: operandos a e b de 32 bits
        a, b       : in  std_logic_vector(31 downto 0);

        -- entrada: sinal de controle que define a operacao
        aluControl : in  std_logic_vector(2 downto 0);

        -- saida: resultado da operacao em 32 bits
        result     : out std_logic_vector(31 downto 0);

        -- saida: flag que indica se o resultado eh zero
        zero       : out std_logic
    );
end entity alu;


architecture behavior of alu is
    -- sinal interno para armazenar o resultado temporariamente
    signal res_s : std_logic_vector(31 downto 0);
    
begin
    -- processo combinacional sensivel as entradas de operandos e de controle
    process(a, b, aluControl)
    begin
        -- seleciona a operacao a ser executada com base no sinal de controle
        case aluControl is
            -- adicao
            when "000" => 
                res_s <= std_logic_vector(unsigned(a) + unsigned(b));
            -- subtracao
            when "001" =>
                res_s <= std_logic_vector(unsigned(a) - unsigned(b));
            -- operacao logica and bit a bit
            when "010" =>
                res_s <= a and b;
            -- operacao logica or bit a bit
            when "011" =>
                res_s <= a or b;
            -- operacao logica xor bit a bit
            when "100" =>
                res_s <= a xor b;
            -- comparacao 'set on less than' com sinal
            when "101" =>
                -- se 'a' for menor que 'b' (interpretados como numeros com sinal)
                if signed(a) < signed(b) then
                    -- o resultado eh 1 (em 32 bits)
                    res_s <= std_logic_vector(to_unsigned(1, 32));
                else
                    -- o resultado eh 0
                    res_s <= (others => '0');
                end if;
            -- para qualquer codigo de controle desconhecido
            when others =>
                -- a saida eh 'x' (indefinido) para indicar um erro
                res_s <= (others => 'X');
        end case;
    end process;

    -- atribui o resultado do sinal interno a porta de saida principal
    result <= res_s;

    -- gera a flag 'zero': sera '1' se o resultado for 0, e '0' caso contrario
    zero <= '1' when unsigned(res_s) = 0 else '0';

end architecture behavior;
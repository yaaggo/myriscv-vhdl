library ieee;
use ieee.std_logic_1164.all;

entity extend is
    port(
        -- entrada: o campo imediato "bruto" de 25 bits da instrucao
        imm    : in  std_logic_vector(24 downto 0);
        
        -- entrada: sinal de controle que define o formato do imediato
        immSrc : in  std_logic_vector(1 downto 0);
        
        -- saida: o valor imediato de 32 bits com extensao de sinal
        immExt : out std_logic_vector(31 downto 0) 
    );  
end extend;

architecture behavior of extend is
begin
    -- processo combinacional sensivel ao imediato bruto e ao seletor
    process(imm, immSrc) begin
        -- seleciona o tipo de extensao com base no sinal de controle immSrc
        case immSrc is
            -- formato i (loads, addi, jalr)
            when "00" =>
                -- estende o sinal do bit mais significativo (imm(24)) e concatena com o imediato de 12 bits
                immExt <= (31 downto 12 => imm(24)) & imm(24 downto 13);
                
            -- formato s (stores)
            when "01" =>
                -- remonta o imediato tipo-s e estende o sinal
                immExt <= (31 downto 12 => imm(24)) & imm(24 downto 18) & imm(4 downto 0);
                
            -- formato b (branches)
            when "10" =>
                -- remonta o imediato tipo-b, estende o sinal e adiciona o '0' no final
                immExt <= (31 downto 12 => imm(24)) & imm(0) & imm(23 downto 18) & imm(4 downto 1) & '0';
                
            -- formato j (jal)
            when "11" =>
                -- remonta o imediato tipo-j, estende o sinal e adiciona o '0' no final
                immExt <= (31 downto 20 => imm(24)) & imm(12 downto 5) & imm(13) & imm(23 downto 14) & '0';
                
            -- para qualquer outro caso, a saida eh indefinida ('-')
            when others =>
                immExt <= (31 downto 0 => '-');
                
        end case;    
    end process ;    
end;
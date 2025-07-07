library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity registers is
    port (
        clk: in  std_logic;                     -- entrada: sinal de clock do sistema
        rst: in  std_logic;                     -- entrada: sinal de reset
        we : in  std_logic;                     -- entrada: habilitacao de escrita (write enable)
        a1 : in  std_logic_vector(4 downto 0);  -- entrada: endereco do primeiro registrador para leitura (rs1)
        a2 : in  std_logic_vector(4 downto 0);  -- entrada: endereco do segundo registrador para leitura (rs2)
        a3 : in  std_logic_vector(4 downto 0);  -- entrada: endereco do registrador para escrita (rd)
        wd : in  std_logic_vector(31 downto 0); -- entrada: dado de 32 bits a ser escrito (write data)
        rd1: out std_logic_vector(31 downto 0); -- saida: dado lido do primeiro registrador (read data 1)
        rd2: out std_logic_vector(31 downto 0)  -- saida: dado lido do segundo registrador (read data 2)
    );
end entity registers;

architecture behavioral of registers is
    -- define um novo tipo: um array de 32 vetores de 32 bits para a memoria
    type reg_array_t is array (0 to 31) of std_logic_vector(31 downto 0);
    
    -- declara o sinal que representa o banco de registradores fisico e o zera
    signal regs : reg_array_t := (others => (others => '0'));
    
begin
    -- processo 1: escrita Sincrona no Banco de Registradores
    write_proc: process(clk, rst)
    begin
        -- se o reset esta ativo (assincrono)
        if rst = '1' then
            -- zera todos os 32 registradores um por um
            for i in 0 to 31 loop
                regs(i) <= (others => '0');
            end loop;
        -- senao, na borda de subida do clock
        elsif rising_edge(clk) then
            -- se a escrita esta habilitada e os sinais sao validos...
            if we = '1' and not is_x(a3) and not is_x(wd) then
                -- ... e o registrador de destino nao eh o x0 (que eh sempre zero)
                if to_integer(unsigned(a3)) /= 0 then
                    -- escreve o dado 'wd' no registrador enderecado por 'a3'
                    regs(to_integer(unsigned(a3))) <= wd;
                end if;
            end if;
        end if;
    end process write_proc;

    -- processo 2: primeira Porta de Leitura (Combinacional)
    process(a1, regs)
    begin
        -- se o endereco de leitura for desconhecido, a saida eh zero
        if is_x(a1) then
            rd1 <= (others => '0');
        else
            -- le o conteudo do registrador 'a1' e o envia para a saida rd1
            rd1 <= regs(to_integer(unsigned(a1)));
        end if;
    end process;
    
    -- processo 3: segunda Porta de Leitura (Combinacional)
    process(a2, regs)
    begin
        -- se o endereco de leitura for desconhecido, a saida eh zero
        if is_x(a2) then
            rd2 <= (others => '0');
        else
            -- le o conteudo do registrador 'a2' e o envia para a saida rd2
            rd2 <= regs(to_integer(unsigned(a2)));
        end if;
    end process;

end architecture behavioral;
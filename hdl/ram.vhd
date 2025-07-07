library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ram is
    -- parametros genericos para configuracao da ram
    generic(
        DATA_WIDTH    : integer := 8;     -- largura do dado em cada endereco (8 bits = 1 byte)
        ADDRESS_WIDTH : integer := 16;    -- largura do barramento de endereco (16 bits)
        DEPTH         : integer := 65536  -- profundidade da memoria (2^16 = 65536)
    );
    port (
        clk: in  std_logic;                     -- entrada: sinal de clock para sincronizar a escrita
        a  : in  std_logic_vector(31 downto 0); -- entrada: endereco de 32 bits para leitura ou escrita
        wd : in  std_logic_vector(31 downto 0); -- entrada: dado de 32 bits a ser escrito (write data)
        we : in  std_logic;                     -- entrada: habilitacao de escrita (write enable)
        rd : out std_logic_vector(31 downto 0)  -- saida: dado de 32 bits lido do endereco (read data)
    );
end entity ram;

architecture behavior of ram is
    -- define um tipo para o array da memoria (array de bytes)
    type ram_type is array (0 to DEPTH-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    
    -- declara o sinal fisico da memoria de dados (dmem) e o zera
    signal dmem : ram_type := (others => (others => '0'));
    
    -- sinal para guardar o endereco convertido para inteiro
    signal base_addr_int : natural range 0 to DEPTH-1;

begin
    -- processo 1: conversao do endereco de entrada (combinacional)
    process(a)
    begin
        -- checa se o endereco eh desconhecido para seguranca da simulacao
        if is_x(a(ADDRESS_WIDTH-1 downto 0)) then
            base_addr_int <= 0;
        else
            -- converte os bits de endereco para um numero inteiro
            base_addr_int <= to_integer(unsigned(a(ADDRESS_WIDTH-1 downto 0)));
        end if;
    end process;


    -- processo 2: escrita sincrona na memoria
    write_proc: process(clk)
    begin
        -- executa na borda de subida do clock
        if rising_edge(clk) then
            -- se a escrita estiver habilitada e os sinais forem validos
            if we = '1' and not is_x(a) and not is_x(wd) then
                -- checa se o endereco eh valido para escrever 4 bytes sem estourar a memoria
                if base_addr_int <= (DEPTH - 4) then
                    -- quebra a palavra de 32 bits em 4 bytes e escreve em enderecos consecutivos
                    dmem(base_addr_int)   <= wd(7 downto 0);   -- byte 0 (menos significativo)
                    dmem(base_addr_int+1) <= wd(15 downto 8);  -- byte 1
                    dmem(base_addr_int+2) <= wd(23 downto 16); -- byte 2
                    dmem(base_addr_int+3) <= wd(31 downto 24); -- byte 3 (mais significativo)
                end if;
            end if;
        end if;
    end process write_proc;


    -- processo 3: leitura assincrona (combinacional) da memoria
    process(base_addr_int, dmem)
    begin
        -- checa se o endereco eh valido para ler 4 bytes
        if base_addr_int <= (DEPTH - 4) then
            -- le 4 bytes de enderecos consecutivos e os monta em uma palavra de 32 bits
            rd(7 downto 0)   <= dmem(base_addr_int);
            rd(15 downto 8)  <= dmem(base_addr_int+1);
            rd(23 downto 16) <= dmem(base_addr_int+2);
            rd(31 downto 24) <= dmem(base_addr_int+3);
        else
            -- se o endereco de leitura for invalido, a saida eh zero para evitar erros
            rd <= (others => '0');
        end if;
    end process;

end architecture behavior;
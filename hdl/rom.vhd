library ieee ;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;
use std.textio.all;
use ieee.std_logic_textio.all;

entity rom is
    -- parametros genericos para configuracao da rom
    generic(
        DATA_WIDTH    : integer := 8;     -- largura do dado em cada endereco (8 bits = 1 byte)
        ADDRESS_WIDTH : integer := 16;    -- largura do barramento de endereco (16 bits)
        DEPTH         : integer := 65536  -- profundidade da memoria (2^16 = 65536 enderecos)
    );
    -- portas de entrada e saida da entidade
    port(
        a  : in  std_logic_vector(31 downto 0); -- entrada: endereco de 32 bits para leitura
        rd : out std_logic_vector(31 downto 0)  -- saida: dado de 32 bits lido do endereco
    );
end entity rom;

architecture behavior of rom is
    -- define um novo tipo de dado: um array para a memoria
    type rom_type is array (0 to DEPTH-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    
    -- declara o sinal da memoria de instrucoes (imem) e o inicializa com zeros
    signal imem : rom_type := (others => (others => '0'));
    
    -- flag para controlar se a memoria ja foi inicializada
    signal initialized : boolean := false;
 
begin
    -- processo para inicializar a memoria a partir de um arquivo
    InitMem: process
        use std.textio.all;
        file f: text;
        variable l: line;
        variable value: std_logic_vector(DATA_WIDTH-1 downto 0);
        variable i: integer := 0;
        variable file_status: file_open_status;
    begin
        -- tenta abrir o arquivo binario em modo de leitura
        file_open(file_status, f, "bitstream.bin", read_mode);
        
        -- se o arquivo foi aberto com sucesso
        if file_status = open_ok then
            -- enquanto nao for o fim do arquivo e a memoria nao estiver cheia
            while not endfile(f) and i < DEPTH loop
                readline(f, l);         -- le uma linha do arquivo
                read(l, value);         -- le o valor da linha para a variavel
                imem(i) <= value;       -- escreve o valor na memoria de instrucoes
                i := i + 1;
            end loop;
            file_close(f);
        -- se o arquivo nao pode ser aberto
        else
            -- preenche a memoria com instrucoes nop (no-operation)
            for j in 0 to DEPTH-1 loop
                if j mod 4 = 0 then
                    imem(j) <= x"13"; -- byte 0 de nop (0x00000013)
                elsif j mod 4 = 1 then
                    imem(j) <= x"00"; -- byte 1 de nop
                elsif j mod 4 = 2 then
                    imem(j) <= x"00"; -- byte 2 de nop
                else
                    imem(j) <= x"00"; -- byte 3 de nop
                end if;
            end loop;
        end if;
        
        initialized <= true; -- sinaliza que a inicializacao terminou
        wait;                -- pausa o processo para que ele rode apenas uma vez
    end process InitMem;


    -- processo principal de leitura da rom (combinacional)
    process(a, initialized)
        variable base_addr_int : natural;
    begin
        -- se a memoria ainda nao foi inicializada, saida eh zero
        if not initialized then
            rd <= (others => '0');
        -- se o endereco for desconhecido ('x'), saida eh zero
        elsif is_x(a(ADDRESS_WIDTH-1 downto 0)) then
            rd <= (others => '0');
        else
            -- converte o endereco para um inteiro
            base_addr_int := to_integer(unsigned(a(ADDRESS_WIDTH-1 downto 0)));
            
            -- verifica se o endereco eh valido para ler 4 bytes
            if base_addr_int <= (DEPTH - 4) then
                -- le os 4 bytes da memoria e os monta na saida de 32 bits
                rd(7 downto 0)   <= imem(base_addr_int);
                rd(15 downto 8)  <= imem(base_addr_int + 1);
                rd(23 downto 16) <= imem(base_addr_int + 2);
                rd(31 downto 24) <= imem(base_addr_int + 3);
            else
                -- saida eh zero para evitar leitura fora dos limites
                rd <= (others => '0');
            end if;
        end if;
    end process;

end behavior;
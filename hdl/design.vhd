library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity design is
port (
    -- portas de entrada principais
    clk : in  std_logic; -- entrada: clock principal do sistema
    rst : in  std_logic; -- entrada: reset principal do sistema

    -- portas de saida para depuracao
    debug_pc          : out std_logic_vector(31 downto 0); -- saida de depuracao: program counter atual
    debug_instr       : out std_logic_vector(31 downto 0); -- saida de depuracao: instrucao atual
    debug_reg_write   : out std_logic;                     -- saida de depuracao: sinal regwrite
    debug_reg_addr    : out std_logic_vector(4 downto 0);  -- saida de depuracao: endereco do registrador de destino
    debug_reg_data    : out std_logic_vector(31 downto 0)  -- saida de depuracao: dado escrito no registrador
);
end entity;

architecture behavior of design is
    -- declaracao de todos os sinais internos ("fios") que conectam os componentes

    -- sinais do caminho de dados do pc
    signal pc_current         : std_logic_vector(31 downto 0); -- pc da instrucao atual
    signal pc_plus4           : std_logic_vector(31 downto 0); -- pc + 4
    signal pc_target          : std_logic_vector(31 downto 0); -- endereco de destino para desvios/saltos
    signal branch_target_addr : std_logic_vector(31 downto 0); -- endereco calculado pelo somador de desvio

    -- sinais da instrucao e seus campos decodificados
    signal instr   : std_logic_vector(31 downto 0); -- a instrucao de 32 bits
    signal opcode  : std_logic_vector(6 downto 0);  -- campo opcode
    signal rd      : std_logic_vector(4 downto 0);  -- campo registrador de destino
    signal funct3  : std_logic_vector(2 downto 0);  -- campo funct3
    signal rs1     : std_logic_vector(4 downto 0);  -- campo registrador fonte 1
    signal rs2     : std_logic_vector(4 downto 0);  -- campo registrador fonte 2
    signal funct7  : std_logic_vector(6 downto 0);  -- campo funct7
    signal imm25   : std_logic_vector(24 downto 0); -- campo imediato bruto

    -- sinais do caminho de dados principal
    signal reg_data1    : std_logic_vector(31 downto 0); -- dado lido do registrador rs1
    signal reg_data2    : std_logic_vector(31 downto 0); -- dado lido do registrador rs2
    signal write_data   : std_logic_vector(31 downto 0); -- dado final a ser escrito de volta
    signal imm_ext      : std_logic_vector(31 downto 0); -- imediato com extensao de sinal
    signal alu_result   : std_logic_vector(31 downto 0); -- resultado da ula
    signal alu_operand2 : std_logic_vector(31 downto 0); -- segundo operando da ula
    signal mem_data_out : std_logic_vector(31 downto 0); -- dado lido da memoria de dados

    -- sinais de controle gerados pelo controller
    signal zero       : std_logic; -- flag 'zero' da ula
    signal resultSrc  : std_logic_vector(1 downto 0);
    signal immSrc     : std_logic_vector(1 downto 0);
    signal aluControl : std_logic_vector(2 downto 0);
    signal memWrite   : std_logic;
    signal aluSrc     : std_logic;
    signal regWrite   : std_logic;
    signal PCSrc      : std_logic;
    signal TargetSrc  : std_logic;
    
begin
    -- conexoes das portas de depuracao
    debug_pc          <= pc_current;
    debug_instr       <= instr;
    debug_reg_write   <= regWrite;
    debug_reg_addr    <= rd;
    debug_reg_data    <= write_data;

    -- instancia a unidade de busca de instrucao (ifetch)
    u_ifetch: entity work.ifetch
        port map (
            clk_i        => clk,        -- conecta o clock principal
            rst_i        => rst,        -- conecta o reset principal
            pc_src_i     => PCSrc,      -- entrada de controle para selecionar o proximo pc
            pc_target_i  => pc_target,  -- entrada com o endereco de destino do desvio/salto
            pc_current_o => pc_current, -- saida com o pc atual
            pc_plus4_o   => pc_plus4    -- saida com o pc + 4
        );

    -- instancia a memoria de instrucoes (rom)
    u_rom: entity work.rom 
        port map (
            a  => pc_current, -- o endereco de leitura eh o pc atual
            rd => instr       -- a saida eh a instrucao
        );

    -- instancia o decodificador de instrucao
    u_decode: entity work.decode 
        port map (
            instr  => instr,  -- entrada eh a instrucao lida da rom
            opcode => opcode, -- saidas com os campos da instrucao
            rd     => rd, 
            funct3 => funct3, 
            rs1    => rs1, 
            rs2    => rs2, 
            funct7 => funct7, 
            imm25  => imm25
        );

    -- instancia a unidade de controle
    u_controller: entity work.controller
        port map (
            op         => opcode,   -- entradas com os campos da instrucao e a flag zero
            funct3     => funct3,
            funct7     => funct7,
            zero       => zero,
            resultSrc  => resultSrc, -- saidas com todos os sinais de controle
            memWrite   => memWrite,
            PCSrc      => PCSrc,
            aluSrc     => aluSrc,
            regWrite   => regWrite,
            immSrc     => immSrc,
            aluControl => aluControl,
            TargetSrc  => TargetSrc
        );

    -- instancia o banco de registradores
    u_regs: entity work.registers 
        port map (
            clk => clk,        -- clock e reset
            rst => rst, 
            we  => regWrite,   -- habilitacao de escrita vinda do controller
            a1  => rs1,        -- endereco de leitura 1
            a2  => rs2,        -- endereco de leitura 2
            a3  => rd,         -- endereco de escrita
            wd  => write_data, -- dado a ser escrito (vem do mux de write-back)
            rd1 => reg_data1,  -- saida com o dado do registrador 1
            rd2 => reg_data2   -- saida com o dado do registrador 2
        );

    -- instancia a unidade de extensao de sinal
    u_ext: entity work.extend 
        port map (
            imm    => imm25,  -- entrada com o imediato bruto
            immSrc => immSrc, -- entrada de controle que define o formato
            immExt => imm_ext -- saida com o imediato de 32 bits estendido
        );

    -- multiplexador para o segundo operando da ula
    u_alu_operand_mux: entity work.mux232
        port map (
            d0 => reg_data2,    -- se alusrc = '0', usa o dado do registrador 2
            d1 => imm_ext,      -- se alusrc = '1', usa o imediato estendido
            s  => aluSrc,       -- sinal de selecao vindo do controller
            y  => alu_operand2  -- saida conectada a ula
        );

    -- instancia a unidade logica e aritmetica (ula)
    u_alu: entity work.alu 
        port map (
            a          => reg_data1,    -- primeiro operando vem sempre do registrador 1
            b          => alu_operand2, -- segundo operando vem do mux
            aluControl => aluControl,   -- sinal de controle que define a operacao
            result     => alu_result,   -- saida com o resultado da operacao
            zero       => zero          -- saida com a flag 'zero'
        );

    -- instancia a memoria de dados (ram)
    u_ram: entity work.ram 
        port map (
            clk => clk,          -- clock para a escrita sincrona
            a   => alu_result,   -- endereco para leitura/escrita eh o resultado da ula
            wd  => reg_data2,    -- dado para escrita vem sempre do registrador 2
            we  => memWrite,     -- habilitacao de escrita vinda do controller
            rd  => mem_data_out  -- saida com o dado lido da memoria
        );

    -- multiplexador (4-para-1) para selecionar o dado a ser escrito no banco de registradores (write-back)
    with resultSrc select
        write_data <= alu_result   when "00", -- se resultsrc = "00", fonte eh o resultado da ula
                      mem_data_out when "01", -- se resultsrc = "01", fonte sao os dados da memoria
                      pc_plus4     when "10", -- se resultsrc = "10", fonte eh pc+4 (para jal/jalr)
                      (others => '0') when others;

    -- somador para calcular o endereco de destino dos desvios (branches)
    u_branch_adder: entity work.adder32
        port map (
            a => pc_current, -- soma o pc atual...
            b => imm_ext,    -- ...com o imediato de desvio
            s => branch_target_addr
        );

    -- multiplexador para selecionar o endereco de destino final para o pc
    with TargetSrc select
        pc_target <= branch_target_addr when '0', -- para beq, jal, etc.
                     alu_result         when '1', -- para jalr
                     (others => '0')    when others;

end architecture;
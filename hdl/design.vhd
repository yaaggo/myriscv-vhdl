library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity design is
port (
    clk : in std_logic;
    rst : in std_logic;
    debug_pc : out std_logic_vector(31 downto 0);
    debug_instr : out std_logic_vector(31 downto 0);
    debug_reg_write : out std_logic;
    debug_reg_addr : out std_logic_vector(4 downto 0);
    debug_reg_data : out std_logic_vector(31 downto 0)
);
end entity;

architecture behavior of design is
    -- Sinais existentes
    signal pc_current, pc_plus4, pc_target : std_logic_vector(31 downto 0);
    signal instr : std_logic_vector(31 downto 0);
    signal opcode : std_logic_vector(6 downto 0);
    signal rd : std_logic_vector(4 downto 0);
    signal funct3 : std_logic_vector(2 downto 0);
    signal rs1 : std_logic_vector(4 downto 0);
    signal rs2 : std_logic_vector(4 downto 0);
    signal funct7 : std_logic_vector(6 downto 0);
    signal imm25 : std_logic_vector(24 downto 0);
    signal reg_data1, reg_data2 : std_logic_vector(31 downto 0);
    signal write_data : std_logic_vector(31 downto 0);
    signal imm_ext : std_logic_vector(31 downto 0);
    signal alu_result : std_logic_vector(31 downto 0);
    signal alu_operand2: std_logic_vector(31 downto 0);
    signal zero : std_logic;
    signal mem_data_out : std_logic_vector(31 downto 0);
    signal resultSrc : std_logic_vector(1 downto 0);
    signal immSrc : std_logic_vector(1 downto 0);
    signal aluControl : std_logic_vector(2 downto 0);
    signal memWrite : std_logic;
    signal aluSrc : std_logic;
    signal regWrite : std_logic;
    signal PCSrc : std_logic;

    signal branch_target_addr : std_logic_vector(31 downto 0);
    signal TargetSrc          : std_logic;
    
begin
    debug_pc <= pc_current;
    debug_instr <= instr;
    debug_reg_write <= regWrite;
    debug_reg_addr <= rd;
    debug_reg_data <= write_data;

    u_ifetch: entity work.ifetch
        port map (
            clk_i        => clk,
            rst_i        => rst,
            pc_src_i     => PCSrc,
            pc_target_i  => pc_target,
            pc_current_o => pc_current,
            pc_plus4_o   => pc_plus4
        );

    u_rom: entity work.rom port map (a => pc_current, rd => instr);

    u_decode: entity work.decode port map (instr => instr, opcode => opcode, rd => rd, funct3 => funct3, rs1 => rs1, rs2 => rs2, funct7 => funct7, imm25 => imm25);

    u_controller: entity work.controller
        port map (
            op         => opcode,
            funct3     => funct3,
            funct7     => funct7,
            zero       => zero,
            resultSrc  => resultSrc,
            memWrite   => memWrite,
            PCSrc      => PCSrc,
            aluSrc     => aluSrc,
            regWrite   => regWrite,
            immSrc     => immSrc,
            aluControl => aluControl,
            TargetSrc  => TargetSrc
        );

    u_regs: entity work.registers port map (clk => clk, rst => rst, we => regWrite, a1 => rs1, a2 => rs2, a3 => rd, wd => write_data, rd1 => reg_data1, rd2 => reg_data2);

    u_ext: entity work.extend port map (imm => imm25, immSrc => immSrc, immExt => imm_ext);

    u_alu_operand_mux: entity work.mux232
    port map (
        d0 => reg_data2,    -- Conectado a 'd0' quando aluSrc = '0'
        d1 => imm_ext,      -- Conectado a 'd1' quando aluSrc = '1'
        s  => aluSrc,       -- Sinal de seleção
        y  => alu_operand2  -- Saída conectada à entrada da ULA
    );

    u_alu: entity work.alu port map (a => reg_data1, b => alu_operand2, aluControl => aluControl, result => alu_result, zero => zero);

    u_ram: entity work.ram port map (clk => clk, a => alu_result, wd => reg_data2, we => memWrite, rd => mem_data_out);

    with resultSrc select
        write_data <= alu_result    when "00",
                      mem_data_out  when "01",
                      pc_plus4      when "10",
                      (others => '0') when others;

    u_branch_adder: entity work.adder32
        port map (
            a => pc_current,
            b => imm_ext,
            s => branch_target_addr
        );

    with TargetSrc select
        pc_target <= branch_target_addr when '0', -- Para BEQ, JAL, etc.
                     alu_result         when '1', -- Para JALR
                     (others => '0')    when others;

end architecture;
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decode is
    port (
        instr    : in  std_logic_vector(31 downto 0); -- entrada: a instrucao de 32 bits
        opcode   : out std_logic_vector(6 downto 0);  -- saida: campo opcode (bits 6-0)
        rd       : out std_logic_vector(4 downto 0);  -- saida: registrador de destino (bits 11-7)
        funct3   : out std_logic_vector(2 downto 0);  -- saida: campo de funcao de 3 bits (14-12)
        rs1      : out std_logic_vector(4 downto 0);  -- saida: primeiro registrador fonte (19-15)
        rs2      : out std_logic_vector(4 downto 0);  -- saida: segundo registrador fonte (24-20)
        funct7   : out std_logic_vector(6 downto 0);  -- saida: campo de funcao de 7 bits (31-25)
        imm25    : out std_logic_vector(24 downto 0)  -- saida: campo imediato "bruto" (31-7)
    );
end entity;

architecture behavior of decode is
begin
    -- conecta os bits 6 a 0 da instrucao a saida opcode
    opcode <= instr(6 downto 0);

    -- conecta os bits 11 a 7 da instrucao a saida rd
    rd <= instr(11 downto 7);

    -- conecta os bits 14 a 12 da instrucao a saida funct3
    funct3 <= instr(14 downto 12);

    -- conecta os bits 19 a 15 da instrucao a saida rs1
    rs1 <= instr(19 downto 15);

    -- conecta os bits 24 a 20 da instrucao a saida rs2
    rs2 <= instr(24 downto 20);

    -- conecta os bits 31 a 25 da instrucao a saida funct7
    funct7 <= instr(31 downto 25);

    -- conecta os bits 31 a 7 da instrucao a saida imm25
    imm25 <= instr(31 downto 7);
    
end architecture;
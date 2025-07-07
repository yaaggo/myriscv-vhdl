library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decode is
    port (
        instr   : in  std_logic_vector(31 downto 0);
        opcode  : out std_logic_vector(6 downto 0);
        rd      : out std_logic_vector(4 downto 0);
        funct3  : out std_logic_vector(2 downto 0);
        rs1     : out std_logic_vector(4 downto 0);
        rs2     : out std_logic_vector(4 downto 0);
        funct7  : out std_logic_vector(6 downto 0);
        imm25   : out std_logic_vector(24 downto 0)
    );
end entity;

architecture behavior of decode is
begin
    opcode  <= instr(6 downto 0);
    rd      <= instr(11 downto 7);
    funct3  <= instr(14 downto 12);
    rs1     <= instr(19 downto 15);
    rs2     <= instr(24 downto 20);
    funct7  <= instr(31 downto 25);
    imm25   <= instr(31 downto 7); 
end architecture;

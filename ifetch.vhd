library ieee;
use ieee.std_logic_1164.all;

entity ifetch is
    port (
        clk_i         : in  std_logic;                     
        rst_i         : in  std_logic;                     
        pc_src_i      : in  std_logic;                     
        pc_target_i   : in  std_logic_vector(31 downto 0); 
        pc_current_o  : out std_logic_vector(31 downto 0);
        pc_plus4_o    : out std_logic_vector(31 downto 0)
    );
end entity ifetch;

architecture behavior of ifetch is
    signal pc_reg_out_s       : std_logic_vector(31 downto 0);
    signal pc_plus_4_s        : std_logic_vector(31 downto 0);
    signal pc_next_mux_out_s  : std_logic_vector(31 downto 0);
    signal constant_4_s       : std_logic_vector(31 downto 0) := x"00000004";

begin
    pc_register_unit: entity work.rreg32
        port map (
            clk => clk_i,
            rst => rst_i,
            d   => pc_next_mux_out_s,
            q   => pc_reg_out_s
        );

    adder_pcplus4_unit: entity work.adder32
        port map (
            a => pc_reg_out_s,
            b => constant_4_s,
            s => pc_plus_4_s
        );

    mux_next_pc_unit: entity work.mux232
        port map (
            d0 => pc_plus_4_s,
            d1 => pc_target_i,
            s  => pc_src_i,
            y  => pc_next_mux_out_s
        );

    pc_current_o <= pc_reg_out_s;
    pc_plus4_o <= pc_plus_4_s;
end architecture behavior;
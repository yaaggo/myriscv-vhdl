library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity testbench is
end testbench;

architecture test of testbench is
    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    
    constant clk_period : time := 10 ns;
    signal sim_finished : boolean := false;

begin
    uut: entity work.design
        port map (
            clk => clk,
            rst => rst
        );
    clk_process : process
    begin
        while not sim_finished loop
            clk <= '0';
            wait for clk_period / 2;
            clk <= '1';
            wait for clk_period / 2;
        end loop;
        wait;
    end process;
    stim_proc: process
    begin
        rst <= '1';
        wait for 30 ns;
        rst <= '0';
        
        report "=== INICIANDO SIMULAcaO ===" severity note;
        report "Reset liberado, processador iniciando execucao..." severity note;
        wait for 1000 ns;
        
        report "=== SIMULAcaO CONCLUiDA COM SUCESSO ===" severity note;
        sim_finished <= true;
        wait for 100 ns;
        
        report "Fim da simulacao - SUCESSO" severity note;
        wait;
    end process;

end architecture test;
library ieee;
use ieee.std_logic_1164.all;

entity ifetch is
    port (
        clk_i         : in  std_logic;                       -- Clock
        rst_i         : in  std_logic;                       -- Reset
        pc_src_i      : in  std_logic;                       -- Seletor de fonte para o próximo PC
        pc_target_i   : in  std_logic_vector(31 downto 0);   -- Endereço alvo para desvios/saltos
        pc_current_o  : out std_logic_vector(31 downto 0);   -- Endereço do PC atual
        pc_plus4_o    : out std_logic_vector(31 downto 0)    -- Endereço do PC atual + 4
    );
end entity ifetch;

architecture behavior of ifetch is
    -- sinais internos que conectam os componentes
    signal pc_reg_out_s      : std_logic_vector(31 downto 0); -- saida do registrador do pc
    signal pc_plus_4_s       : std_logic_vector(31 downto 0); -- resultado de pc + 4
    signal pc_next_mux_out_s : std_logic_vector(31 downto 0); -- saida do mux que define o proximo pc
    signal constant_4_s      : std_logic_vector(31 downto 0) := x"00000004"; -- constante de 4 bytes

begin
    -- unidade 1: o registrador que armazena o pc
    pc_register_unit: entity work.rreg32
        port map (
            clk => clk_i,                 -- clock do sistema
            rst => rst_i,                 -- reset do sistema
            d   => pc_next_mux_out_s,     -- entrada: o proximo valor do pc
            q   => pc_reg_out_s           -- saida: o valor atual do pc
        );

    -- unidade 2: o somador que calcula pc + 4
    adder_pcplus4_unit: entity work.adder32
        port map (
            a => pc_reg_out_s,    -- entrada a: o pc atual
            b => constant_4_s,    -- entrada b: a constante 4
            s => pc_plus_4_s      -- saida s: o resultado da soma
        );

    -- unidade 3: o multiplexador que decide o proximo pc
    mux_next_pc_unit: entity work.mux232
        port map (
            d0 => pc_plus_4_s,       -- entrada 0: pc + 4 (execucao normal)
            d1 => pc_target_i,       -- entrada 1: endereco de desvio/salto
            s  => pc_src_i,          -- seletor: sinal de controle do controller
            y  => pc_next_mux_out_s  -- saida: o valor escolhido para ser o proximo pc
        );

    -- conexao dos sinais internos com as portas de saida da entidade
    pc_current_o <= pc_reg_out_s;
    pc_plus4_o   <= pc_plus_4_s;
    
end architecture behavior;
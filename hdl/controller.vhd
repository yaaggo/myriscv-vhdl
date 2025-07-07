library ieee;
use ieee.std_logic_1164.all;

entity controller is
    port(
        op        : in std_logic_vector(6 downto 0);
        funct3    : in std_logic_vector(2 downto 0);
        funct7    : in std_logic_vector(6 downto 0);
        zero      : in std_logic;
        resultSrc : out std_logic_vector(1 downto 0);
        memWrite  : out std_logic;
        PCSrc     : out std_logic;
        aluSrc    : out std_logic;
        regWrite  : out std_logic;
        immSrc    : out std_logic_vector(1 downto 0);
        aluControl: out std_logic_vector(2 downto 0);
        TargetSrc : out std_logic
    );
end entity controller;

architecture behavior of controller is
    signal aluOp    : std_logic_vector(1 downto 0);
    signal branch   : std_logic;
    signal jump     : std_logic;
    signal bne      : std_logic;
    signal beq      : std_logic;
    signal RtypeSub : std_logic;
begin

    PCSrc <= (zero and beq) or ((not zero) and bne) or jump;

    mainDecoder: process(op, funct3)
    begin
        branch    <= '0';
        jump      <= '0';
        beq       <= '0';
        bne       <= '0';
        resultSrc <= "XX"; -- "Don't care" por padrão
        memWrite  <= '0';
        aluSrc    <= 'X';  -- "Don't care" por padrão
        regWrite  <= '0';
        immSrc    <= "XX"; -- "Don't care" por padrão
        aluOp     <= "XX"; -- "Don't care" por padrão
        TargetSrc <= '0';

        case op is
            when "0000011" =>  -- lw (load word)
                resultSrc <= "01"; -- Resultado vem da memória
                aluSrc    <= '1';  -- ALU usa imediato
                regWrite  <= '1';  -- Habilita escrita no registrador
                immSrc    <= "00"; -- Imediato tipo I
                aluOp     <= "00"; -- ULA deve somar

            when "0100011" =>  -- sw (store word)
                memWrite  <= '1';  -- Habilita escrita na memória
                aluSrc    <= '1';  -- ALU usa imediato
                immSrc    <= "01"; -- Imediato tipo S
                aluOp     <= "00"; -- ULA deve somar

            when "0110011" =>  -- R-type (add, sub, etc.)
                resultSrc <= "00"; -- Resultado vem da ULA
                aluSrc    <= '0';  -- ALU usa outro registrador
                regWrite  <= '1';  -- Habilita escrita no registrador
                aluOp     <= "10"; -- ULA é controlada pelo funct3/funct7

            when "1100011" =>  -- Branch (beq, bne)
                branch    <= '1';
                TargetSrc <= '0';
                if funct3 = "000" then -- beq
                    beq   <= '1';
                else -- bne
                    bne   <= '1';
                end if;
                aluSrc    <= '0';
                immSrc    <= "10";
                aluOp     <= "01";

            when "0010011" =>  -- I-type ALU (addi, etc.)
                resultSrc <= "00"; -- Resultado vem da ULA
                aluSrc    <= '1';  -- ALU usa imediato
                regWrite  <= '1';  -- Habilita escrita no registrador
                immSrc    <= "00"; -- Imediato tipo I
                aluOp     <= "10"; -- ULA é controlada pelo funct3

            when "1101111" =>  -- Jal (jump and link)
                jump      <= '1';
                TargetSrc <= '0';
                resultSrc <= "10";
                regWrite  <= '1';
                immSrc    <= "11";
            when "1100111" => -- jalr
                jump      <= '1';
                TargetSrc <= '1';
                resultSrc <= "10";
                regWrite  <= '1';
                immSrc    <= "00";
                aluOp     <= "00";
            
            when others =>
                null;
        end case;
    end process mainDecoder;

    aluDecoder: process (op, funct3, funct7, aluOp)
    begin
        RtypeSub <= funct7(5) and op(5);
        
        case aluOp is
            when "00" => -- adição para lw, sw
                aluControl <= "000"; -- add

            when "01" => -- subtração para beq, bne
                aluControl <= "001"; -- sub

            when "10" => -- para tipos R e I (ALU)
                case funct3 is
                    when "000" =>  -- add, addi, sub
                        if RtypeSub = '1' then
                            aluControl <= "001"; -- sub
                        else
                            aluControl <= "000"; -- add ou addi
                        end if;
                    when "010" =>  -- slt, slti
                        aluControl <= "101";
                    when "100" =>  -- xor, xori
                        aluControl <= "100";
                    when "110" =>  -- or, ori
                        aluControl <= "011";
                    when "111" =>  -- and, andi
                        aluControl <= "010";
                    when others => -- desconhecido
                        aluControl <= "---";
                end case;
            
            when others => -- desconhecido
                aluControl <= "---";
        end case;
    end process aluDecoder;

end architecture behavior;
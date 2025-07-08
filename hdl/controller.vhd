library ieee;
use ieee.std_logic_1164.all;

entity controller is
    port(
        -- entradas que definem a instrucao e o estado
        op        : in  std_logic_vector(6 downto 0);  -- entrada: campo opcode da instrucao
        funct3    : in  std_logic_vector(2 downto 0);  -- entrada: campo funct3 da instrucao
        funct7    : in  std_logic_vector(6 downto 0);  -- entrada: campo funct7 da instrucao
        zero      : in  std_logic;                     -- entrada: flag 'zero' vinda da ula

        -- saidas: sinais de controle para o caminho de dados
        resultSrc : out std_logic_vector(1 downto 0);  -- saida: seleciona a fonte do dado para escrita
        memWrite  : out std_logic;                     -- saida: habilita a escrita na memoria de dados
        PCSrc     : out std_logic;                     -- saida: seleciona a fonte do proximo pc
        aluSrc    : out std_logic;                     -- saida: seleciona a segunda fonte de operando para a ula
        regWrite  : out std_logic;                     -- saida: habilita a escrita no banco de registradores
        immSrc    : out std_logic_vector(1 downto 0);  -- saida: seleciona o tipo do imediato para a unidade de extensao
        aluControl: out std_logic_vector(2 downto 0);  -- saida: define a operacao da ula
        TargetSrc : out std_logic                      -- saida: seleciona a fonte do endereco de destino para saltos
    );
end entity controller;

architecture behavior of controller is
    -- sinais internos para comunicacao entre os processos de decodificacao
    signal aluOp    : std_logic_vector(1 downto 0); -- sinal de controle interno para o decodificador da alu
    signal branch   : std_logic;                    -- sinal que indica uma instrucao de branch
    signal jump     : std_logic;                    -- sinal que indica uma instrucao de jump
    signal bne      : std_logic;                    -- sinal que indica uma instrucao 'branch on not equal'
    signal beq      : std_logic;                    -- sinal que indica uma instrucao 'branch on equal'

begin
    -- logica concorrente para o sinal de controle do pc (PCSrc)
    PCSrc <= (zero and beq) or ((not zero) and bne) or jump;


    -- processo 1: decodificador principal
    -- gera a maioria dos sinais de controle com base no opcode da instrucao
    mainDecoder: process(op, funct3)
    begin
        -- valores padrao (inativos) para os sinais de controle
        branch    <= '0';
        jump      <= '0';
        beq       <= '0';
        bne       <= '0';
        resultSrc <= "XX"; -- "don't care"
        memWrite  <= '0';
        aluSrc    <= 'X';  -- "don't care"
        regWrite  <= '0';
        immSrc    <= "XX"; -- "don't care"
        aluOp     <= "XX"; -- "don't care"
        TargetSrc <= '0';

        -- decodifica a instrucao com base no opcode
        case op is
            -- instrucao lw (load word)
            when "0000011" =>
                resultSrc <= "01"; -- resultado vem da memoria
                aluSrc    <= '1';  -- ula usa imediato
                regWrite  <= '1';  -- habilita escrita no registrador
                immSrc    <= "00"; -- imediato tipo i
                aluOp     <= "00"; -- ula deve somar

            -- instrucao sw (store word)
            when "0100011" =>
                memWrite <= '1';  -- habilita escrita na memoria
                aluSrc   <= '1';  -- ula usa imediato
                immSrc   <= "01"; -- imediato tipo s
                aluOp    <= "00"; -- ula deve somar

            -- instrucoes tipo-r (add, sub, etc.)
            when "0110011" =>
                resultSrc <= "00"; -- resultado vem da ula
                aluSrc    <= '0';  -- ula usa outro registrador
                regWrite  <= '1';  -- habilita escrita no registrador
                aluOp     <= "10"; -- operacao da ula eh definida pelo aludecoder

            -- instrucoes branch (beq, bne)
            when "1100011" =>
                branch <= '1';
                if funct3 = "000" then -- beq
                    beq <= '1';
                else -- bne
                    bne <= '1';
                end if;
                aluSrc <= '0';
                immSrc <= "10"; -- imediato tipo b
                aluOp  <= "01"; -- ula deve subtrair

            -- instrucoes tipo-i (addi, etc.)
            when "0010011" =>
                resultSrc <= "00"; -- resultado vem da ula
                aluSrc    <= '1';  -- ula usa imediato
                regWrite  <= '1';  -- habilita escrita no registrador
                immSrc    <= "00"; -- imediato tipo i
                aluOp     <= "10"; -- operacao da ula eh definida pelo aludecoder

            -- instrucao jal (jump and link)
            when "1101111" =>
                jump      <= '1';
                resultSrc <= "10"; -- resultado eh pc + 4
                regWrite  <= '1';
                immSrc    <= "11"; -- imediato tipo j
                
            -- instrucao jalr
            when "1100111" =>
                jump      <= '1';
                TargetSrc <= '1';  -- endereco de destino vem da ula
                resultSrc <= "10"; -- resultado eh pc + 4
                regWrite  <= '1';
                immSrc    <= "00"; -- imediato tipo i
                aluOp     <= "00"; -- ula deve somar

            when others =>
                null; -- nao faz nada para opcodes desconhecidos
        end case;
    end process mainDecoder;


    -- processo 2: fecodificador da ULA
    -- gera o sinal aluControl final com base no aluOp, funct3 e funct7
    aluDecoder: process (op, funct3, funct7, aluOp)
    	-- declaracao como variavel porque como signal ele nao atualiza a tempo
    	variable RtypeSub : std_logic;
    begin
        -- logica para detectar a instrucao sub do tipo-r
        RtypeSub := funct7(5) and op(5);

        case aluOp is
            -- caso 1: adicao para lw, sw, jalr
            when "00" =>
                aluControl <= "000"; -- add

            -- caso 2: subtracao para beq, bne
            when "01" =>
                aluControl <= "001"; -- sub

            -- caso 3: operacao depende de funct3 (para tipo-r e tipo-i)
            when "10" =>
                case funct3 is
                    when "000" => -- add, addi, sub
                        if RtypeSub = '1' then
                            aluControl <= "001"; -- sub
                        else
                            aluControl <= "000"; -- add ou addi
                        end if;
                    when "010" => -- slt, slti
                        aluControl <= "101";
                    when "100" => -- xor, xori
                        aluControl <= "100";
                    when "110" => -- or, ori
                        aluControl <= "011";
                    when "111" => -- and, andi
                        aluControl <= "010";
                    when others =>
                        aluControl <= "---"; -- desconhecido
                end case;

            when others =>
                aluControl <= "---"; -- desconhecido
        end case;
    end process aluDecoder;

end architecture behavior;
#  Processador MyRISC-V de 32 bits em VHDL

## Sumário

  * [Introdução](#introdução)
  * [Arquitetura e Organização do Processador](#arquitetura-e-organização-do-processador)
      * [1. Busca de Instrução (Instruction Fetch - IF)](#1-busca-de-instrução-instruction-fetch---if)
      * [2. Decodificação de Instrução (Instruction Decode - ID)](#2-decodificação-de-instrução-instruction-decode---id)
      * [3. Execução (Execute - EX)](#3-execução-execute---ex)
      * [4. Acesso à Memória (Memory - MEM)](#4-acesso-à-memória-memory---mem)
      * [5. Escrita de Volta (Write-Back - WB)](#5-escrita-de-volta-write-back---wb)
  * [Estrutura dos Arquivos](#estrutura-dos-arquivos)

## Introdução

Este repositório contém a implementação completa de um **processador de 32 bits com um subconjunto da arquitetura RISC-V**, apelidado pelo professor de MyRISC-V, desenvolvido inteiramente em VHDL. O design é **monocíclo**, o que significa que cada instrução é executada completamente em um único ciclo de clock.

## Arquitetura e Organização do Processador

Apesar de ser uma implementação de ciclo único, a arquitetura é projetada seguindo a lógica dos cinco estágios clássicos de um pipeline de processador. Abaixo, cada estágio é explicado e vinculado aos componentes de código VHDL correspondentes.

### 1\. Busca de Instrução (Instruction Fetch - IF)

Esta é a primeira fase, responsável por buscar a próxima instrução a ser executada da memória de instruções. O endereço da instrução está contido no Program Counter (PC).

#### Componentes de Código Envolvidos:

___

#### **`ifetch.vhd`**: 
A entidade principal desta fase. Contém o registrador do PC, a lógica para incrementá-lo em 4 a cada ciclo (`PC + 4`) e um multiplexador para decidir se o próximo PC será `PC + 4` ou um endereço de destino de um desvio/salto.

```vhdl
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

```
* **clk_i** : É o sinal de clock principal do sistema. Todas as operações síncronas, como a atuualização do program counter, ocorrem na borda de suubida deste sinal.
* **rst_i**: O sinal de reset. Quando ativado, ele força o PC a um estado inicial conhecido, que geralmente é o endereço da primeira instrução do programa (no caso, ``x"00000000"``).
* **pc_src_i**: Este é um sinal de controle de 1 bit quee atua como seeletor para um multiplexador interno. Ele determina qual será o endereço da próxima instrução:
    * Se ``pc_src_i`` for ``'0'`` o PC será atualizado com o endereço da instrução seeeguinte em sequência.
    * Se ``pc_src_i`` for ``'1'``, o PC será atualizado com o endereço de destino (``pc_target_i``), o que ocorre durante uma instrução de desvio ou salto.
* **pc_target_i**: Um vetor de ee32 bits que fornece o endereço de destino para uma operação de desvio ou salto. Este valor é calculado fora da unidade ``ifetch`` e é carregado no PC sse ``pc_src_i`` for ``'1'``.
* **pc_target_o**: Fornece o endereço da instrução atual para o resto do processador. Este endereço é usado principalmente para ler a instrução correspondente na memória de instruções (ROM).
* **pc_plus4_o**: Esta saída pré-calcula e fornece o endereço da próxima instrução sequencial (pc + 4). Este valor tem dois usos principais:
    * é o valor padrão para a atualização do PC no próximo ciclo de clock.
    * é usado para calcular o endereço de retorno em instruções como jal (jump and link).

```vhdl
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
```
* **pc_register_unit (registrador do PC)**: Este é o componente central da unidade, uma instância da entidade ``rreg32``. Ele funciona como o **program counter**, armazenando fisicamente o endereço de 32 bits da instrução que está sendo executada no momento. A cada ciclo de clock, ele é atualizado com um novo valor vindo da entrada ``d``, que é determinado pelo multiplexador. Sua saída ``q`` representa o endereço da instrução atual.
* **adder_pcplus4_unit**: Esta instância da entidade ``adder32`` tem a tarefa de calcular o endereço da próxima instrução sequencial. Ele pega a saída atual do PC (``pc_reg_out_s``), soma com a constante 4 e gera o resultado ``pc_plus_4_s``. Esse cálculo é continuo e necessário porque as instruções têm 4 bytes de comprimento.
* **mux_next_pc_unit**: Este multiplexador, que é instância de ``mux232`` é o responsável pela tomada de decisão. Ele escolhe qual será o próximo valor a ser carregado no PC. A escolha é controlada pelo sinal ``pc_src_i``:
    * Se ``pc_src_i = '0'``, o multiplexador seleciona sua entrada ``d0``, que é o valor de ``pc + 4``. Isso corresponde à execução normal e sequencial do programa.
    * Se ``pc_src_i = '1'``, ele seleciona a entrada ``d1``, que é o ``pc_target_i``. Isso acontece quando o processador precisa executar quando o processador precisa executar um desvio (branch) ou um salto (jump), alterando o fluxo de controle para um novo endereço.

As duas últimas linhas do código simplesmente conectam os sinais internos gerados pelo registrador e pelo somador às portas de saída da entidade ``ifetch``, tornando esses valores disponíveis para outras partes do processador.
___
### **`rom.vhd`**: 
A entidade rom (Read-Only Memory) é um componente que armazena o programa no qual o processador irá executar. Ela funciona como uma memória somente de leitura que, ao receber um endereço do ``ifetch``, retorna a instrução de 32 bits correspondente.

```vhdl
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

```
* **generic**: Os genéricos tornam o componente configurável.
    * **DATA_WIDTH := 8**: Define que cada posição individual na memória armazena um dado de 8 bits.
    * **ADDRESS_WIDTH := 16**: Define que o endereço usado para acessar a memória tem 16 bits.
    * **DEPTH := 65536** Define o número total de endereços de memória. Este valor é diretamente derivado da largura do endereço (2^16 = 65536).
* **a**: Esta é a porta de endereço de entrada. O processador (através do ``ifetch``) coloca aqui o endereço de 32 bits da instrução que deseja ler. Internamente, a ROM usará apenas os 16 bits inferiores deste vetor (``a(15 downto 0)``) para selecionar a posição na memória, conforme definido pelo ``ADDRESS_WIDTH``.
* **rd**: Esta é a porta de dados de saída. Após receber um endereço em ``a``, a ROM lê o conteúdo e o disponibiliza nesta porta. Como as instruções RISC-V têm 32 biits e a memória é organizada em bytes (``DATA_WIDTH = 8``), a ROM internamente lê 4 bytes consecutivos a partir do endereço base e os concatena para formar a instrução completa de 32 bits que é enviada para o processador.

``` vhdl
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
        file_open(file_status, f, "C:\Users\yagog\Documents\coding\riscv\bitstream.bin", read_mode);
        
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

```

**1. Processo ``InitMem``**: Este processo é projetado para rodar apenas uma vez no início da simulação. A instrução ``wait;`` no final garante que ele seja suspenso indefinidamente após a primeira execução. Sua função é popular a memória (``imem``);
- Ele tenta abrir o arquivo especificado no caminho. Se o arquivo for encontrado, ele lê cada linha (que deve conter um valor de 8 bits) e armazena na memória ``imem`` sequencialmente.
- Se o arquivo não for encontrado, o processo não falha. Em vez disso, ele preenche a memória com a instrução ``NOP`` do RISC-V.
- Ao final, ele define a flag ``initialized`` como ``true``, informando ao outro processo que a memória está pronta para ser lida.

**2. Processo de Leitura**: Este processo é ativado sempre que o endereço de entrada ``a`` ou a flag ``initialized`` mudam. Ele é responsável por ler os dados da memória:
- Primeiro, ele verifica se a memória já foi inicializada. Se não, ele envia ``0`` na saída para evitar que o processo leia lixo.
- Ele pega os 16 bits menos significativos do endereço de entrada ``a`` e os converte em um número inteiro (``base_addr_int``), que servirá de índice para o array ``imem``.
- Como o processador lê palavras de 32 bits, mas a memória é organizada em bytes, o processo lê quatro posíções consecutivas da memória a partir do ``base_addr_int``.
- Ele monta a palavra de 32 bits na porta de saída ``rd`` concatenando os quatro bytes lidos na ordem correta (little-endian).
___
### **`rreg32.vhd`**: 

Um registrador genérico de 32 bits, usado dentro do `ifetch` para armazenar o valor atual do PC.

___
### **`adder32.vhd`**: 
O somador é usado aqui para duas finalidades: calcular `PC + 4` dentro do `ifetch` e, no `design` principal, para calcular o endereço de destino para instruções de desvio (`pc_current + imm_ext`).
___

### 2\. Decodificação de Instrução (Instruction Decode - ID)

Nesta fase, a instrução buscada é decodificada para determinar qual operação deve ser executada. Os operandos são lidos do banco de registradores e os sinais de controle para as fases seguintes são gerados.

#### Componentes de Código Envolvidos:

  * **`decode.vhd`**: Recebe a instrução de 32 bits e a divide em seus campos constituintes (`opcode`, `rs1`, `rs2`, `rd`, `funct3`, `funct7`, `imm`).
  * **`controller.vhd`**: A unidade de controle. Ela recebe os campos `opcode`, `funct3` e `funct7` e gera todos os sinais de controle necessários para o resto do processador (`regWrite`, `memWrite`, `aluSrc`, `aluControl`, `PCSrc`, etc.).
  * **`registers.vhd`**: O banco de registradores. Usa os campos `rs1` e `rs2` como endereços para ler os valores dos operandos, que são disponibilizados em suas saídas `rd1` e `rd2`.
  * **`extend.vhd`**: A unidade de extensão de sinal. Recebe o campo de imediato (`imm`) da instrução e o estende para 32 bits, de acordo com o formato da instrução (I, S, B ou J), determinado pelo sinal `immSrc` do controlador.

### 3\. Execução (Execute - EX)

A fase de execução é onde a computação principal da instrução ocorre. A Unidade Lógica e Aritmética (ULA) realiza a operação matemática ou lógica necessária.

#### Componentes de Código Envolvidos:

  * **`alu.vhd`**: A Unidade Lógica e Aritmética. Recebe os operandos (um do banco de registradores e outro que pode ser de um registrador ou o imediato estendido) e realiza a operação determinada pelo sinal `aluControl`. Ela gera o resultado e um sinal `zero`, usado para desvios condicionais.
  * **`mux232.vhd`**: Um multiplexador no `design.vhd` (`alu_operand2 <= ...`) seleciona o segundo operando para a ULA. A decisão é baseada no sinal `aluSrc` do controlador.

### 4\. Acesso à Memória (Memory - MEM)

Esta fase é responsável por interagir com a memória de dados. Ela é utilizada apenas por instruções de carga (`lw`) e armazenamento (`sw`).

#### Componentes de Código Envolvidos:

  * **`ram.vhd`**: A memória de dados (RAM). Para uma instrução `sw`, ela escreve o dado de um registrador na posição de memória indicada pelo resultado da ULA. Para uma instrução `lw`, ela lê o dado da memória e o disponibiliza para a próxima fase. O sinal `memWrite` do controlador determina se a operação é de escrita.

### 5\. Escrita de Volta (Write-Back - WB)

Na fase final, o resultado da operação (seja o resultado da ULA ou o dado lido da memória) é escrito de volta no banco de registradores.

#### Componentes de Código Envolvidos:

  * **`registers.vhd`**: A porta de escrita do banco de registradores é usada aqui. O endereço do registrador de destino (`rd`) e o dado a ser escrito (`wd`) são fornecidos, e a escrita é habilitada pelo sinal `regWrite`.
  * **`mux332.vhd` (implementado com `with resultSrc select` no `design.vhd`)**: Este multiplexador seleciona qual dado será escrito no banco de registradores. As opções são: o resultado da ULA, o dado lido da memória ou o valor de `PC + 4` (para a instrução `jal`).

## Estrutura dos Arquivos

```
├── adder32.vhd      # Somador de 32 bits.
├── alu.vhd          # Unidade Lógica e Aritmética.
├── controller.vhd   # Unidade de Controle principal que gera os sinais.
├── decode.vhd       # Decodificador de instruções.
├── design.vhd       # Entidade de topo, conecta todos os componentes.
├── extend.vhd       # Unidade de extensão de sinal para imediatos.
├── ifetch.vhd       # Unidade de busca de instrução (contém o PC).
├── mux232.vhd       # Multiplexador 2-para-1 de 32 bits.
├── mux332.vhd       # Multiplexador 3-para-1 de 32 bits.
├── ram.vhd          # Memória de Dados (RAM).
├── registers.vhd    # Banco de Registradores (32 registradores de 32 bits).
├── rom.vhd          # Memória de Instruções (ROM).
└── rreg32.vhd       # Registrador genérico de 32 bits.
```

-----

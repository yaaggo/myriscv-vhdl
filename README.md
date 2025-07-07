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
* **`clk_i`** : É o sinal de clock principal do sistema. Todas as operações síncronas, como a atuualização do program counter, ocorrem na borda de suubida deste sinal.
* **`rst_i`**: O sinal de reset. Quando ativado, ele força o PC a um estado inicial conhecido, que geralmente é o endereço da primeira instrução do programa (no caso, ``x"00000000"``).
* **`pc_src_i`**: Este é um sinal de controle de 1 bit quee atua como seeletor para um multiplexador interno. Ele determina qual será o endereço da próxima instrução:
    * Se ``pc_src_i`` for ``'0'`` o PC será atualizado com o endereço da instrução seeeguinte em sequência.
    * Se ``pc_src_i`` for ``'1'``, o PC será atualizado com o endereço de destino (``pc_target_i``), o que ocorre durante uma instrução de desvio ou salto.
* **`pc_target_i`**: Um vetor de ee32 bits que fornece o endereço de destino para uma operação de desvio ou salto. Este valor é calculado fora da unidade ``ifetch`` e é carregado no PC sse ``pc_src_i`` for ``'1'``.
* **`pc_target_o`**: Fornece o endereço da instrução atual para o resto do processador. Este endereço é usado principalmente para ler a instrução correspondente na memória de instruções (ROM).
* **`pc_plus4_o`**: Esta saída pré-calcula e fornece o endereço da próxima instrução sequencial (pc + 4). Este valor tem dois usos principais:
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
* **`pc_register_unit (registrador do PC)`**: Este é o componente central da unidade, uma instância da entidade ``rreg32``. Ele funciona como o **program counter**, armazenando fisicamente o endereço de 32 bits da instrução que está sendo executada no momento. A cada ciclo de clock, ele é atualizado com um novo valor vindo da entrada ``d``, que é determinado pelo multiplexador. Sua saída ``q`` representa o endereço da instrução atual.
* **`adder_pcplus4_unit`**: Esta instância da entidade ``adder32`` tem a tarefa de calcular o endereço da próxima instrução sequencial. Ele pega a saída atual do PC (``pc_reg_out_s``), soma com a constante 4 e gera o resultado ``pc_plus_4_s``. Esse cálculo é continuo e necessário porque as instruções têm 4 bytes de comprimento.
* **`mux_next_pc_unit`**: Este multiplexador, que é instância de ``mux232`` é o responsável pela tomada de decisão. Ele escolhe qual será o próximo valor a ser carregado no PC. A escolha é controlada pelo sinal ``pc_src_i``:
    * Se ``pc_src_i = '0'``, o multiplexador seleciona sua entrada ``d0``, que é o valor de ``pc + 4``. Isso corresponde à execução normal e sequencial do programa.
    * Se ``pc_src_i = '1'``, ele seleciona a entrada ``d1``, que é o ``pc_target_i``. Isso acontece quando o processador precisa executar quando o processador precisa executar um desvio (branch) ou um salto (jump), alterando o fluxo de controle para um novo endereço.

As duas últimas linhas do código simplesmente conectam os sinais internos gerados pelo registrador e pelo somador às portas de saída da entidade ``ifetch``, tornando esses valores disponíveis para outras partes do processador.
___
#### **`rom.vhd`**: 
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
    * **`DATA_WIDTH := 8`**: Define que cada posição individual na memória armazena um dado de 8 bits.
    * **`ADDRESS_WIDTH := 16`**: Define que o endereço usado para acessar a memória tem 16 bits.
    * **`DEPTH := 65536`** Define o número total de endereços de memória. Este valor é diretamente derivado da largura do endereço (2^16 = 65536).
* **`a`**: Esta é a porta de endereço de entrada. O processador (através do ``ifetch``) coloca aqui o endereço de 32 bits da instrução que deseja ler. Internamente, a ROM usará apenas os 16 bits inferiores deste vetor (``a(15 downto 0)``) para selecionar a posição na memória, conforme definido pelo ``ADDRESS_WIDTH``.
* **`rd`**: Esta é a porta de dados de saída. Após receber um endereço em ``a``, a ROM lê o conteúdo e o disponibiliza nesta porta. Como as instruções RISC-V têm 32 biits e a memória é organizada em bytes (``DATA_WIDTH = 8``), a ROM internamente lê 4 bytes consecutivos a partir do endereço base e os concatena para formar a instrução completa de 32 bits que é enviada para o processador.

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
        file_open(file_status, f, "bitstream.bin", read_mode);
        
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
#### **`rreg32.vhd`**: 

Um registrador genérico de 32 bits, usado dentro do `ifetch` para armazenar o valor atual do PC.

```vhdl
entity rreg32 is
    port(
        clk : in  std_logic;                       -- entrada: sinal de clock do sistema
        rst : in  std_logic;                       -- entrada: sinal de reset para inicializacao
        d   : in  std_logic_vector(31 downto 0);   -- entrada: dado de 32 bits a ser armazenado
        q   : out std_logic_vector(31 downto 0)    -- saida: dado de 32 bits atualmente armazenado
    );
end rreg32;

```
- **`clk`**: Porta de clock. Este é um sinal de entrada que sincroniza a operação do registrador. A captura do novo dado ocorre especificamente na borda de subida deste sinal (max usa borda de descida, mas todas as pesquisas que fiz, é usado borda de subida).

- **`rst`**: A porta de reset (reinicialização). Quando este sinal de entrada é ativado, ele força o registrador a um estado padrão predefinido (geralmente todos os bits em '0'), independentemente do clock ou da entrada d. É fundamental para garantir que o sistema comece em um estado conhecido.

- **`d`**: Esta é a porta de dados de entrada (data). O valor presente nesta porta é o que será capturado e armazenado dentro do registrador na próxima borda de subida do clock (se o reset não esteja ativo).

- **`q`**: A porta de saída. Esta porta expõe continuamente o valor que está atualmente armazenado dentro do registrador. Seu valor só muda após a borda de subida do clock ou durante um reset.

```vhdl
architecture behavior of rreg32 is
begin
    -- define um processo sensivel aos sinais de clock e reset
    process(clk, rst)
    begin
        -- checa se o sinal de reset esta ativo (reset assincrono)
        if (rst = '1') then
            -- se o reset estiver ativo, a saida 'q' eh zerada
            q <= (others => '0');
        -- senao, na borda de subida do clock...
        elsif (rising_edge(clk)) then
            -- a saida 'q' recebe o valor da entrada 'd'
            q <= d;
        end if;
    end process;
end behavior;

```
- **`process(clk, rst)`**: A lista de sensibilidade informa ao simulador que o código dentro do processo só deve ser executado quando houver uma mudança nos sinais `clk` ou `rst`.
- **`if (rst = '1') then`**: A primeira condição verificada é a do reset. Se o sinal ``rst`` estiver em nivel lógico alto, a saída `q` é imediatamente forçada para zero. A expressão `(others => '0')` é uma forma de atribuir '0' a todos os 32 bits do vetor `q`. Como essa ação ocorre independentemente do clock, da para chamar de reset assíncrono.
- **`elsif (rising_edge(clk)) then`**: Se o reset não estiver ativo, o processo então verifica se ocorreu uma borda de subida no sinal de clock. A função `rising_edge()`detecta exatamente isso.
- **`q <= d`**: Apenas no instante da borda de subida do clock, o valor presente na entrada `d`é registrado e atribuído à saída `q`. Em todos os outros momentos, `q` mantém seu valor anterior, efetivamente "lembrando" o dado (funcionamento basicamente igual ao de um flip-flop D).
___
#### **`adder32.vhd`**: 
O somador é usado aqui para duas finalidades: calcular `PC + 4` dentro do `ifetch` e, no `design` principal, para calcular o endereço de destino para instruções de desvio (`pc_current + imm_ext`).

```vhdl
entity adder32 is
    port(
        a : in  std_logic_vector(31 downto 0); -- entrada a: primeiro operando de 32 bits
        b : in  std_logic_vector(31 downto 0); -- entrada b: segundo operando de 32 bits
        s : out std_logic_vector(31 downto 0)  -- saida s: resultado da soma de a + b
    );
end adder32;
```
- **`a`**: A primeira entrada de 32 bits para operação de soma.
- **`b`**: A segunda entrada de 32 bits para operação de soma.
- **`s`**: A saída de 32 bits que contém o resultado da coma de `a + b` (não tem tratamento de overflow).

```vhdl
architecture behavior of adder32 is
begin
    -- atribui a 's' o resultado da soma de 'a' e 'b'
    -- e necessario converter os tipos para realizar a operacao aritmetica
    s <= std_logic_vector(unsigned(a) + unsigned(b));
end behavior;
```
- **`s <= std_logic_vector(unsigned(a) + unsigned(b))`**:
    - ``unsigned(a)`` e ``unsigned(b)``: As portas de entrada `a` e ``b`` são do tipo ``std_logic_vector``, que para o VHDL é apenas uma coleção de bits (``'0'``, ``'1'``, ``'X'``, etc.) sem significado numérico. Para realizar uma operação aritmética como a soma, precisamos primeiro dizer ao compilador como interpretar esses bits. A função ``unsigned()`` (da biblioteca ieee.numeric_std) faz exatamente isso: ela converte (faz um "type cast") os vetores de bits para o tipo ``unsigned``, que representa um número inteiro sem sinal.
    - ``... + ...``: Uma vez que ``a`` e ``b`` são tratados como números do tipo ``unsigned``, o operador de adição ``+`` pode ser aplicado a eles da maneira que esperamos.
    - ``std_logic_vector(...)``: O resultado da soma ainda é do tipo ``unsigned``. No entanto, a porta de saída ``s`` é do tipo ``std_logic_vector``. Portanto, é necessário fazer a conversão inversa, transformando o resultado numérico de volta para um vetor de bits padrão antes de ir para a saída.
    - ``s <= ...``: Este é o operador de atribuição de sinal em VHDL. Ele pega o valor final calculado à direita e o atribui à porta de saída ``s``.
___

### 2\. Decodificação de Instrução (Instruction Decode - ID)

Nesta fase, a instrução buscada é decodificada para determinar qual operação deve ser executada. Os operandos são lidos do banco de registradores e os sinais de controle para as fases seguintes são gerados.

### Componentes de Código Envolvidos:

#### **`decode.vhd`**: 
Recebe a instrução de 32 bits e a divide em seus campos constituintes (`opcode`, `rs1`, `rs2`, `rd`, `funct3`, `funct7`, `imm`).

```vhdl
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
```
- **``instr``**: A instrução de 32 bits a ser decodificada, vinda diretamente da memória de instruções (``rom``).

- **``opcode``**: O código da operação. Este campo de 7 bits (``instr(6 downto 0)``) é a identificação principal da instrução. A Unidade de Controle o utiliza para determinar o formato da instrução (``R``, ``I``, ``S``, etc.) e gerar os principais sinais de controle.

- **``rd``**: O endereço do registrador de destino (``instr(11 downto 7)``). É o registrador que receberá o resultado final da operação.

- **``funct3``**: Um campo de função adicional de 3 bits (``instr(14 downto 12)``). Ele ajuda a diferenciar operações que compartilham o mesmo opcode. Por exemplo, para instruções do ``Tipo-R``, ele distingue entre ``ADD/SUB``, ``SLL``, ``SLT``, etc.

- **``rs1``**: O endereço do primeiro registrador fonte (``instr(19 downto 15)``). Contém o primeiro operando para a ``ULA``.

- **``rs2``**: O endereço do segundo registrador fonte (``instr(24 downto 20)``). Contém o segundo operando para a ULA em instruções do ``Tipo-R`` e ``Tipo-S``.

- **``funct7``**: Um segundo campo de função adicional de 7 bits (``instr(31 downto 25)``). É usado principalmente para distinguir entre operações do ``Tipo-R`` que possuem o mesmo opcode e funct3, como ADD (``funct7 = 0b0000000``) e ``SUB`` (``funct7 = 0b0100000``).

- **``imm25``**: Este campo (``instr(31 downto 7)``) agrupa todos os bits que podem, potencialmente, fazer parte de um valor imediato. Ele não representa um imediato válido por si só, mas sim uma fatia "bruta" da instrução. Este vetor é então enviado para a entidade ``extend``, que sabe como extrair e montar o imediato correto de ``12``, ``20`` ou ``21`` bits, dependendo do tipo da instrução.

```vhdl
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
```
- **``opcode <= instr(6 downto 0)``**: Os 7 bits menos significativos da instrução de entrada (``instr``) são conectados diretamente à porta de saída opcode.

- **``rd <= instr(11 downto 7)``**: Os bits 11 a 7 da ``instr`` são conectados à porta de saída ``rd``.

- **``funct3 <= instr(14 downto 12)``**: Os bits 14 a 12 da ``instr`` são conectados à porta de saída funct3.

- **``rs1 <= instr(19 downto 15)``**: Os bits 19 a 15 da ``instr`` são conectados à porta de saída ``rs1``.

- **``rs2 <= instr(24 downto 20)``**: Os bits 24 a 20 da ``instr`` são conectados à porta de saída ``rs2``.

- **``funct7 <= instr(31 downto 25)``**: Os 7 bits mais significativos da ``instr`` são conectados à porta de saída ``funct7``.

- **``imm25 <= instr(31 downto 7)``**: Uma fatia maior da instrução, dos bits 31 a 7, é conectada à porta de saída ``imm25``.

Em resumo, a entidade decode não processa informação. Ela simplesmente a organiza, expondo os campos da instrução em portas de saída nomeadas e de tamanho apropriado, prontas para serem usadas pela ``controller``, ``registers`` e ``extend``.

___
#### **`controller.vhd`**: 
A entidade ``controller`` é cérebro do processador. Trata-se de um bloco combinacional que funciona como o "maestro" do hardware. Sua função é receber os campos decodificados da instrução (``opcode``, ``funct3``, ``funct7``) e o estado atual do processador (como a flag zero da ULA) e, com base neles, gerar todos os sinais de controle que ditam o comportamento do caminho de dados para executar aquela instrução específica.

```vhdl
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
```
- **``op, funct3, funct7``**: Estes três campos, vindos diretamente do decodificador decode, definem unicamente a instrução a ser executada. O ``controller`` usa uma combinação desses valores para determinar todos os outros sinais de controle.
- **``zero``**: Esta entrada de 1 bit é uma flag de estado vinda da ULA. Ela é ``'1'`` se o resultado da última operação da ULA foi zero. Seu uso principal é para instruções de desvio condicional como beq, que só desvia se a subtração dos dois operandos resultar em zero.

- **``resultSrc``**: Controla um multiplexador que decide qual resultado será escrito de volta no banco de registradores. As fontes podem ser: o resultado da ULA, o dado lido da memória de dados (``ram``), ou o endereço ``PC + 4`` (para instruções de salto e link).

- **``memWrite``**: Um sinal de habilitação. Quando ``'1'``, permite a escrita na memória de dados (``ram``). Ativado apenas para instruções de armazenamento (``sw``).

- **``PCSrc``**: Controla o multiplexador que seleciona o próximo endereço para o Program Counter (``PC``). Se ``'0'``, o PC recebe ``PC + 4``. Se ``'1'``, ele recebe o endereço de destino de um desvio ou salto.

- **``aluSrc``**: Controla o multiplexador na segunda entrada da ULA. Se ``'0'``, a ULA recebe o valor do segundo registrador fonte (``rs2``). Se ``'1'``, ela recebe o valor imediato de 32 bits já com a extensão de sinal.

- **``regWrite``**: Outro sinal de habilitação. Quando ``'1'``, permite que o resultado seja escrito no banco de registradores. É desativado para instruções que não geram resultados a serem salvos, como ``sw`` e desvios.

- **``immSrc``**: Controla a unidade extend. Este sinal informa qual formato de imediato (``I``, ``S``, ``B`` ou ``J``) deve ser usado para interpretar e estender o valor imediato da instrução.

- **``aluControl``**: Um vetor de 3 bits que comanda diretamente a ULA, dizendo qual operação ela deve realizar (soma, subtração, AND, OR, etc.).

- **``TargetSrc``**: Um sinal de controle mais específico para saltos. Ele seleciona a fonte do endereço de destino para o PC, diferenciando entre um alvo calculado com base no PC (para ``jal``) e um alvo calculado com base em um registrador (para ``jalr``).


```vhdl
architecture behavior of controller is
    -- sinais internos para comunicacao entre os processos de decodificacao
    signal aluOp    : std_logic_vector(1 downto 0); -- sinal de controle interno para o decodificador da alu
    signal branch   : std_logic;                    -- sinal que indica uma instrucao de branch
    signal jump     : std_logic;                    -- sinal que indica uma instrucao de jump
    signal bne      : std_logic;                    -- sinal que indica uma instrucao 'branch on not equal'
    signal beq      : std_logic;                    -- sinal que indica uma instrucao 'branch on equal'
    signal RtypeSub : std_logic;                    -- sinal para diferenciar entre add e sub no tipo-r

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
    begin
        -- logica para detectar a instrucao sub do tipo-r
        RtypeSub <= funct7(5) and op(5);

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
```

##### 1. Lógica Concorrente para PCSrc

A linha ``PCSrc <= (zero and beq) or ((not zero) and bne) or jump;`` é uma atribuição que executa continuamente. Ela define a lógica de desvio e salto:

- O sinal ``PCSrc`` será ``'1'`` (indicando um desvio/salto) se:

    - A instrução for um ``beq`` **E** a flag ``zero`` da ULA for ``'1'``.

    - **OU** a instrução for um ``bne`` **E** a flag ``zero`` for ``'0'``.

    - **OU** a instrução for um ``jal`` ou ``jalr`` (sinal ``jump`` é ``'1'``).

- Caso contrário, ``PCSrc`` será ``'0'``, indicando execução sequencial (``PC + 4``).

##### 2. Processo mainDecoder
Este processo olha para o ``opcode`` da instrução e, através de um grande ``case``, define a maioria dos sinais de controle.

- **Valores Padrão**: No início do processo, todos os sinais são definidos para um estado "inativo" ou "não importa" (``'0'`` ou ``'X'``). Isso garante que, para qualquer instrução, os sinais que não são explicitamente ativados permaneçam desativados.

- **Decodificação por opcode:** Cada ``when`` no ``case`` corresponde a um tipo de instrução. exemplo:

    - Para ``lw``, ele ativa ``regWrite`` e ``aluSrc`` e configura ``resultSrc`` para selecionar a memória.
    - Para ``sw``, ele ativa ``memWrite``.
    - Para instruções do Tipo-R, ele configura o caminho de dados para usar dois registradores (``aluSrc <= '0'``).

- **Sinal ``aluOp``**: Este processo não gera o ``aluControl`` final diretamente. Em vez disso, ele gera um sinal intermediário de 2 bits, ``aluOp``, que pré-classifica a operação da ULA (ex: "00" para soma, "01" para subtração, "10" para operações que dependem do ``funct3``). Isso simplifica a lógica e a divide em dois níveis.

##### 3. Processo aluDecoder
Este segundo processo tem como sua única função gerar o sinal final ``aluControl`` de 3 bits que vai para a ULA.

- **Lógica de Dois Níveis**: Ele usa o ``aluOp`` do decodificador principal como sua entrada principal.

- **Casos Simples**: Se ``aluOp`` for "00" ou "01", ele sabe imediatamente que a operação é ``ADD`` ou ``SUB``.

- **Caso Complexo (``aluOp = "10"``)**: Quando ``aluOp`` é "10", o decodificador sabe que precisa olhar mais a fundo, para o campo ``funct3`` da instrução.

    - Um ``case`` aninhado em ``funct3`` seleciona a operação correta (``slt``, ``xor``, ``or``, ``and``).

    - Para o caso especial de ``funct3 = "000"``, ele precisa diferenciar entre ``add/addi`` e ``sub``. A lógica ``if RtypeSub = '1'`` (que usa o ``funct7``) faz essa distinção final.
___
#### **`registers.vhd`**:
O banco de registradores. Usa os campos `rs1` e `rs2` como endereços para ler os valores dos operandos, que são disponibilizados em suas saídas `rd1` e `rd2`.
___
#### **`extend.vhd`**:
A unidade de extensão de sinal. Recebe o campo de imediato (`imm`) da instrução e o estende para 32 bits, de acordo com o formato da instrução (I, S, B ou J), determinado pelo sinal `immSrc` do controlador.

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

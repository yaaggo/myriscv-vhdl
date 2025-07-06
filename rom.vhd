library ieee ;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;
use std.textio.all;
use ieee.std_logic_textio.all;

entity rom is
generic(
    DATA_WIDTH : integer := 8;
    ADDRESS_WIDTH: integer := 16;
    DEPTH : integer := 65536
);
port(
    a : in std_logic_vector(31 downto 0);
    rd: out std_logic_vector(31 downto 0)
);
end entity rom;

architecture behavior of rom is
    type rom_type is array (0 to DEPTH-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    signal imem : rom_type := (others => (others => '0'));
    signal initialized : boolean := false;
 
begin
    InitMem: process
        use std.textio.all;
        file f: text;
        variable l: line;
        variable value: std_logic_vector(DATA_WIDTH-1 downto 0);
        variable i: integer := 0;
        variable file_status: file_open_status;
    begin
        file_open(file_status, f, "C:\Users\yagog\Documents\coding\riscv\bitstream.bin", read_mode);
        
        if file_status = open_ok then
            while not endfile(f) and i < DEPTH loop
                readline(f, l);
                read(l, value);
                imem(i) <= value;
                i := i + 1;
            end loop;
            file_close(f);
        else
            for j in 0 to DEPTH-1 loop
                if j mod 4 = 0 then
                    imem(j) <= x"13"; -- byte 0 de NOP
                elsif j mod 4 = 1 then
                    imem(j) <= x"00"; -- byte 1 de NOP  
                elsif j mod 4 = 2 then
                    imem(j) <= x"00"; -- byte 2 de NOP
                else
                    imem(j) <= x"00"; -- byte 3 de NOP
                end if;
            end loop;
        end if;
        
        initialized <= true;
        wait;
    end process InitMem;

    process(a, initialized)
        variable base_addr_int : natural;
    begin
        if not initialized then
            rd <= (others => '0');
        elsif is_x(a(ADDRESS_WIDTH-1 downto 0)) then
            rd <= (others => '0');
        else
            base_addr_int := to_integer(unsigned(a(ADDRESS_WIDTH-1 downto 0)));
            if base_addr_int <= (DEPTH - 4) then
                rd(7 downto 0) <= imem(base_addr_int);
                rd(15 downto 8) <= imem(base_addr_int + 1);
                rd(23 downto 16) <= imem(base_addr_int + 2);
                rd(31 downto 24) <= imem(base_addr_int + 3);
            else
                rd <= (others => '0');
            end if;
        end if;
    end process;

end behavior;
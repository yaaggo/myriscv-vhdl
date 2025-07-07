library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ram is
generic(
    DATA_WIDTH : integer := 8;
    ADDRESS_WIDTH : integer := 16;
    DEPTH : integer := 65536
);
port (
    clk: in std_logic;
    a : in std_logic_vector(31 downto 0);
    wd : in std_logic_vector(31 downto 0);
    we : in std_logic;
    rd : out std_logic_vector(31 downto 0)
);
end entity ram;

architecture behavior of ram is
    type ram_type is array (0 to DEPTH-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    signal dmem : ram_type := (others => (others => '0'));
    signal base_addr_int : natural range 0 to DEPTH-1;

begin
    process(a)
    begin
        if is_x(a(ADDRESS_WIDTH-1 downto 0)) then
            base_addr_int <= 0;
        else
            base_addr_int <= to_integer(unsigned(a(ADDRESS_WIDTH-1 downto 0)));
        end if;
    end process;

    write_proc: process(clk)
    begin
        if rising_edge(clk) then
            if we = '1' and not is_x(a) and not is_x(wd) then
                if base_addr_int <= (DEPTH - 4) then
                    dmem(base_addr_int) <= wd(7 downto 0);
                    dmem(base_addr_int+1) <= wd(15 downto 8);
                    dmem(base_addr_int+2) <= wd(23 downto 16);
                    dmem(base_addr_int+3) <= wd(31 downto 24);
                end if;
            end if;
        end if;
    end process write_proc;

    process(base_addr_int, dmem)
    begin
        if base_addr_int <= (DEPTH - 4) then
            rd(7 downto 0) <= dmem(base_addr_int);
            rd(15 downto 8) <= dmem(base_addr_int+1);
            rd(23 downto 16) <= dmem(base_addr_int+2);
            rd(31 downto 24) <= dmem(base_addr_int+3);
        else
            rd <= (others => '0');
        end if;
    end process;

end architecture behavior;
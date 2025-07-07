library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity registers is
port (
    clk: in std_logic;
    rst: in std_logic;
    we : in std_logic;
    a1 : in std_logic_vector( 4 downto 0);
    a2 : in std_logic_vector( 4 downto 0);
    a3 : in std_logic_vector( 4 downto 0);
    wd : in std_logic_vector(31 downto 0);
    rd1: out std_logic_vector(31 downto 0);
    rd2: out std_logic_vector(31 downto 0)
);
end entity registers;

architecture behavioral of registers is
    type reg_array_t is array (0 to 31) of std_logic_vector(31 downto 0);
    signal regs : reg_array_t := (others => (others => '0'));
begin
    write_proc: process(clk, rst)
    begin
        if rst = '1' then
            for i in 0 to 31 loop
                regs(i) <= (others => '0');
            end loop;
        elsif rising_edge(clk) then
            if we = '1' and not is_x(a3) and not is_x(wd) then
                if to_integer(unsigned(a3)) /= 0 then
                    regs(to_integer(unsigned(a3))) <= wd;
                end if;
            end if;
        end if;
    end process write_proc;

    process(a1, regs)
    begin
        if is_x(a1) then
            rd1 <= (others => '0');
        else
            rd1 <= regs(to_integer(unsigned(a1)));
        end if;
    end process;
    
    process(a2, regs)
    begin
        if is_x(a2) then
            rd2 <= (others => '0');
        else
            rd2 <= regs(to_integer(unsigned(a2)));
        end if;
    end process;

end architecture behavioral;
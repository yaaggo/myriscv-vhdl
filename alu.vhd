library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity alu is
    port(
        a, b       : in  std_logic_vector(31 downto 0);
        aluControl : in  std_logic_vector(2 downto 0);
        result     : out std_logic_vector(31 downto 0);
        zero       : out std_logic
    );
end entity alu;

architecture behavior of alu is
    signal res_s : std_logic_vector(31 downto 0);
begin
    process(a, b, aluControl)
    begin
        case aluControl is
            when "000" => -- ADD
                res_s <= std_logic_vector(unsigned(a) + unsigned(b));
            when "001" => -- SUB
                res_s <= std_logic_vector(unsigned(a) - unsigned(b));
            when "010" => -- AND
                res_s <= a and b;
            when "011" => -- OR
                res_s <= a or b;
            when "100" => -- XOR
                res_s <= a xor b;
            when "101" => -- SLT (Set on Less Than)
                if signed(a) < signed(b) then
                    res_s <= std_logic_vector(to_unsigned(1, 32));
                else
                    res_s <= (others => '0');
                end if;
            when others =>
                res_s <= (others => 'X');
        end case;
    end process;

    result <= res_s;

    zero <= '1' when unsigned(res_s) = 0 else '0';

end architecture behavior;
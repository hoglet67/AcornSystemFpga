library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use ieee.std_logic_unsigned.all;

entity keyboard is
    port (
        clock      : in  std_logic;
        nreset     : in  std_logic;
        ps2_clk    : in  std_logic;
        ps2_data   : in  std_logic;
        nbreak_in  : in  std_logic;
        nbreak_out : out std_logic;
        caps_led   : out std_logic;
        shift_led  : out std_logic;
        keyout     : out std_logic_vector(7 downto 0)
        );
end entity;

architecture rtl of keyboard is

    signal keyb_data  : std_logic_vector(7 downto 0);
    signal keyb_valid : std_logic;
    signal keyb_error : std_logic;
    signal release    : std_logic;
    signal extended   : std_logic;
    signal nbreak_in1 : std_logic;
    signal shift      : std_logic;
    signal ctrl       : std_logic;

begin

    ps2 : entity work.ps2_intf port map (
        clock,
        nreset,
        ps2_clk,
        ps2_data,
        keyb_data,
        keyb_valid,
        keyb_error);

    process(clock, nreset)
    variable keytmp: std_logic_vector(7 downto 0);
    begin
        if nreset = '0' then

            shift      <= '0';
            ctrl       <= '0';
            release    <= '0';
            extended   <= '0';
            keyout     <= (others => '1');
            nbreak_out <= '1';
            nbreak_in1 <= nbreak_in;
            
        elsif rising_edge(CLOCK) then

            -- handle the break key seperately, as it's value also depends on BREAK_IN
            if keyb_valid = '1' and keyb_data = X"09" then
                nbreak_out <= release;
            elsif nbreak_in /= nbreak_in1 then
                nbreak_out <= nbreak_in;
            end if;
            nbreak_in1 <= nbreak_in;

            if keyb_valid = '1' then
                if keyb_data = X"e0" then
                    extended <= '1';
                elsif keyb_data = X"f0" then
                    release <= '1';
                else
                    release  <= '0';
                    extended <= '0';

                    keytmp := x"00";
                    
                    case keyb_data is
                        when X"12" | X"59" =>
                            if (extended = '0') then -- Ignore fake shifts
                                shift  <= not release; -- Left SHIFT -- Right SHIFT
                            end if;
                            
                        when X"14" => ctrl <= not release;  -- LEFT/RIGHT CTRL (CTRL)
 
                        when X"6B" => keytmp := X"08";  -- LEFT
                        when X"74" => keytmp := X"09";  -- RIGHT
                        when X"72" => keytmp := X"0A";  -- DOWN
                        when X"75" => keytmp := X"0B";  -- UP
                        when X"5A" => keytmp := X"0D";  -- RETURN
                        when X"58" => keytmp := X"16";  -- CAPS LOCK
                        when X"76" => keytmp := X"1B";  -- ESCAPE

                        when X"29" => keytmp := X"20";  -- SPACE
                        when X"45" => keytmp := X"30";  -- 0
                        when X"16" => keytmp := X"31";  -- 1
                        when X"1E" => keytmp := X"32";  -- 2
                        when X"26" => keytmp := X"33";  -- 3
                        when X"25" => keytmp := X"34";  -- 4
                        when X"2E" => keytmp := X"35";  -- 5
                        when X"36" => keytmp := X"36";  -- 6
                        when X"3D" => keytmp := X"37";  -- 7
                        when X"3E" => keytmp := X"38";  -- 8
                        when X"46" => keytmp := X"39";  -- 9
                        when X"52" => keytmp := X"3A";  -- '   full colon substitute
                        when X"4C" => keytmp := X"3B";  -- ;
                        when X"41" => keytmp := X"2C";  -- ,
                        when X"4E" => keytmp := X"2D";  -- -
                        when X"49" => keytmp := X"2E";  -- .
                        when X"4A" => keytmp := X"2F";  -- /

                        when X"0D" => keytmp := X"40";  -- @ (TAB)
                        when X"1C" => keytmp := X"41";  -- A
                        when X"32" => keytmp := X"42";  -- B
                        when X"21" => keytmp := X"43";  -- C
                        when X"23" => keytmp := X"44";  -- D
                        when X"24" => keytmp := X"45";  -- E
                        when X"2B" => keytmp := X"46";  -- F
                        when X"34" => keytmp := X"47";  -- G
                        when X"33" => keytmp := X"48";  -- H
                        when X"43" => keytmp := X"49";  -- I
                        when X"3B" => keytmp := X"4A";  -- J
                        when X"42" => keytmp := X"4B";  -- K
                        when X"4B" => keytmp := X"4C";  -- L
                        when X"3A" => keytmp := X"4D";  -- M
                        when X"31" => keytmp := X"4E";  -- N
                        when X"44" => keytmp := X"4F";  -- O
                        when X"4D" => keytmp := X"50";  -- P
                        when X"15" => keytmp := X"51";  -- Q
                        when X"2D" => keytmp := X"52";  -- R
                        when X"1B" => keytmp := X"53";  -- S
                        when X"2C" => keytmp := X"54";  -- T
                        when X"3C" => keytmp := X"55";  -- U
                        when X"2A" => keytmp := X"56";  -- V
                        when X"1D" => keytmp := X"57";  -- W
                        when X"22" => keytmp := X"58";  -- X
                        when X"35" => keytmp := X"59";  -- Y
                        when X"1A" => keytmp := X"5A";  -- Z
                        when X"54" => keytmp := X"5B";  -- [
                        when X"5D" => keytmp := X"5C";  -- \
                        when X"5B" => keytmp := X"5D";  -- ]
                        when X"0E" => keytmp := X"5E";  -- ^
                        when X"66" => keytmp := X"7F";  -- BACKSPACE (DELETE)
                        when others => null;
                    end case;
                    if keytmp > x"20" and keytmp < x"40" then
                        if shift = '1' then
                            keytmp := keytmp xor x"10";
                        end if;
                    elsif keytmp >= x"40" then
                        if ctrl = '1' then
                            keytmp := keytmp and x"3F";
                        elsif shift = '1' then
                            keytmp := keytmp xor x"20";
                        end if;
                    end if;
                    if release = '0' then
                        keytmp := keytmp or x"80";
                    end if;
                    keyout <= keytmp;
                end if;
            end if;
        end if;
    end process;

    shift_led  <= shift;
    caps_led   <= ctrl;
    
end architecture;



library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.htm_pkg.all;

entity mult_m_tb is
end entity mult_m_tb;

architecture sim of mult_m_tb is

    signal clk      : std_logic := '0';
    signal reset    : std_logic := '0';
    signal start    : std_logic := '0';
    signal done     : std_logic;
    signal sim_done : boolean := false;

    signal a_in     : matrix_4x4_in := (others => (others => (others => '0')));
    signal b_in     : matrix_4x4_in := (others => (others => (others => '0')));
    signal res_out  : matrix_4x4_out;

    signal b_row0_col1 : signed(15 downto 0);
    signal res_row0_col3 : signed(31 downto 0);
    signal res_row1_col3 : signed(31 downto 0); 

    -- Q2.14 (1.0 = 16384)
    constant IDENTITY_16 : matrix_4x4_in := (
        (to_signed(16384, 16), to_signed(0, 16), to_signed(0, 16), to_signed(0, 16)),
        (to_signed(0, 16), to_signed(16384, 16), to_signed(0, 16), to_signed(0, 16)),
        (to_signed(0, 16), to_signed(0, 16), to_signed(16384, 16), to_signed(0, 16)),
        (to_signed(0, 16), to_signed(0, 16), to_signed(0, 16), to_signed(16384, 16))
    );

begin

    b_row0_col1 <= b_in(0, 1);
    res_row0_col3 <= res_out(0, 3);
    res_row1_col3 <= res_out(1, 3);

    -- (Unit Under Test)
    UUT: entity work.mult_m
        port map (
            clk     => clk,
            reset   => reset,
            start   => start,
            a       => a_in,
            b       => b_in,
            result  => res_out,
            done    => done
        );

    clk_process: process
    begin
        while not sim_done loop
            clk <= '0';
            wait for 5 ns;
            clk <= '1';
            wait for 5 ns;
        end loop;
        wait;
    end process;

    stim_process: process
    begin
        reset <= '1';
        wait for 20 ns;
        reset <= '0';
        wait for 20 ns;

        a_in <= IDENTITY_16;
        a_in(0, 3) <= to_signed(16384, 16); -- Tx = 1.0
        a_in(1, 3) <= to_signed(8192, 16);  -- Ty = 0.5 (16384 / 2)

        b_in <= IDENTITY_16;
        b_in(0, 0) <= to_signed(0, 16);      -- cos(90)
        b_in(0, 1) <= to_signed(-16384, 16); -- -sin(90)
        b_in(1, 0) <= to_signed(16384, 16);  -- sin(90)
        b_in(1, 1) <= to_signed(0, 16);      -- cos(90)

        start <= '1';
        wait for 10 ns; 
        start <= '0';

        wait until done = '1';
        
        wait for 50 ns;

        sim_done <= true;
        wait;
    end process;

end architecture sim;


-- ghdl -a mult_m.vhd               
-- ghdl -a mult_m_tb.vhd            
-- ghdl -e mult_m_tb                
-- ghdl -r mult_m_tb --vcd=ondas.vcd --stop-time=1000ns
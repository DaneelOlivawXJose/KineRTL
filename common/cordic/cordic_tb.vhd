library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cordic_tb is
end entity cordic_tb;

architecture sim of cordic_tb is

    constant DATA_WIDTH : integer := 16;
    constant ITERATIONS : integer := 15;
    
    signal clk      : std_logic := '0';
    signal reset    : std_logic := '0';
    signal start    : std_logic := '0';
    signal angle_in : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    
    signal sin_out  : signed(DATA_WIDTH-1 downto 0);
    signal cos_out  : signed(DATA_WIDTH-1 downto 0);
    signal done     : std_logic;

begin

    -- (UUT - Unit Under Test)
    UUT: entity work.cordic
        generic map (
            DATA_WIDTH => DATA_WIDTH,
            ITERATIONS => ITERATIONS
        )
        port map (
            clk      => clk,
            reset    => reset,
            start    => start,
            angle_in => angle_in,
            sin_out  => sin_out,
            cos_out  => cos_out,
            done     => done
        );

    -- (100 MHz)
    clk_process: process
    begin
        clk <= '0';
        wait for 5 ns;
        clk <= '1';
        wait for 5 ns;
    end process;

    stim_process: process
    begin
        -- Initial reset
        reset <= '1';
        wait for 20 ns;
        reset <= '0';
        wait for 20 ns;
        
        -- 45º (pi/4 = 0.785398 rad)
        -- Scale Q2.14: 0.785398 * 2^14 = 12868
        angle_in <= to_signed(12868, DATA_WIDTH);
        start <= '1';
        wait for 10 ns;
        start <= '0';
        wait until done = '1';
        wait for 50 ns;
        
        -- 30º (pi/6 = 0.523598 rad)
        -- Scale Q2.14: 0.523598 * 2^14 = 8578
        angle_in <= to_signed(8578, DATA_WIDTH);
        start <= '1';
        wait for 10 ns;
        start <= '0';
        wait until done = '1';
        wait for 50 ns;
        
        wait; 
    end process;

end architecture sim;

-- ghdl -a cordic.vhd               
-- ghdl -a cordic_tb.vhd            
-- ghdl -e cordic_tb                
-- ghdl -r cordic_tb --vcd=ondas.vcd --stop-time=1000ns
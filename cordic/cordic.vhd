----------------------------------------------------------------------------------
-- Company: Personal
-- Engineer: Jose Segura Montes
-- 
-- Create Date: 2026
-- Design Name: FPGA-Accelerated N-DOF Industrial Robot Kinematics
-- Module Name: cordic - rtl
-- Project Name: ros2-fpga
-- Target Devices: Generic FPGA (Xilinx/Intel)
-- Tool Versions: GHDL / Vivado / Quartus
-- Description: High-performance, parameterizable VHDL-based Direct Kinematics 
--              solver for a N-DOF industrial robot. Implements a 
--              pipelined architecture using Parallel CORDICs (trigonometric 
--              functions) and a custom Matrix Multiplier (4x4) operating under 
--              a robust Qx.y fixed-point arithmetic system.
--
-- Dependencies: robot_config_pkg
-- 
-- Revision: 13/09/2026
--
-- Copyright (c) 2026 Jose Segura Montes. All rights reserved.
-- This code is licensed under proprietary/open-source terms as applicable.
----------------------------------------------------------------------------------

----------------------------------------------------------------------------------
-- Architecture & Interface Documentation
-- 
-- Entity Name: cordic
-- Purpose: Implements a pipelined CORDIC algorithm for computing trigonometric functions.
--
-- Inputs:
--   * clk      (std_logic)              : System clock driving the internal FSM pipeline.
--   * reset    (std_logic)              : Asynchronous active-high reset signal.
--   * start    (std_logic)              : Single-cycle trigger pulse to initiate computation.
--   * angle_in (fp_type)                : Input angle in fixed-point representation (Qx.y format).
-- 
--
-- Outputs:
--   * sin_out  (fp_type)                : Sine of the input angle.
--   * cos_out  (fp_type)                : Cosine of the input angle.
--   * done     (std_logic)              : Completion flag asserted for one clock cycle.
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.robot_config_pkg.all;

entity cordic is
    generic (
        ITERATIONS : integer := 24;
        DATA_WIDTH : integer := 32
    );
    port (
        clk      : in  std_logic;
        reset    : in  std_logic;
        start    : in  std_logic;
        angle_in : in  fp_type;
        sin_out  : out fp_type;
        cos_out  : out fp_type;
        done     : out std_logic
    );
end entity cordic;

architecture rtl of cordic is
    
    -- FSM Types
    type state_type is (IDLE, CALCULATING, FINISHED);
    signal state : state_type := IDLE;
    
    signal x_reg, y_reg, z_reg : fp_type;
    signal iter_counter        : integer range 0 to ITERATIONS;

    -- Scale factor for fixed-point conversion (2^FRAC_WIDTH)
    constant SCALE_FACTOR : real := 2.0 ** real(FRAC_WIDTH);

    -- Constant K (0.607252935)
    constant K_GAIN : fp_type := to_signed(integer(0.607252935 * SCALE_FACTOR), TOTAL_WIDTH);

    -- atan LUT for CORDIC iterations
    type atan_array_type is array (0 to ITERATIONS-1) of fp_type;
    function init_atan_table return atan_array_type is
        variable table : atan_array_type;
    begin
        for i in 0 to ITERATIONS-1 loop
            -- Utiliza arctan de la librería math_real y escala al punto fijo actual
            table(i) := to_signed(integer(arctan(1.0 / (2.0 ** real(i))) * SCALE_FACTOR), TOTAL_WIDTH);
        end loop;
        return table;
    end function;
    constant ATAN_TABLE : atan_array_type := init_atan_table;

begin
    process(clk, reset)
    begin
        if reset = '1' then
            state <= IDLE;
            done <= '0';
            sin_out <= (others => '0');
            cos_out <= (others => '0');
            x_reg <= (others => '0');
            y_reg <= (others => '0');
            z_reg <= (others => '0');
            iter_counter <= 0;

        elsif rising_edge(clk) then   
            case state is
                when IDLE =>
                    if start = '1' then
                        x_reg <= K_GAIN; 
                        y_reg <= (others => '0');
                        z_reg <= angle_in;
                        iter_counter <= 0;
                        state <= CALCULATING;
                        done <= '0';
                    end if;

                -- Algorithmic CORDIC rotation mode implementation
                when CALCULATING =>
                    if iter_counter = ITERATIONS then
                        state <= FINISHED;
                    else
                        if z_reg(TOTAL_WIDTH-1) = '0' then
                            x_reg <= x_reg - shift_right(y_reg, iter_counter);
                            y_reg <= y_reg + shift_right(x_reg, iter_counter);
                            z_reg <= z_reg - ATAN_TABLE(iter_counter);
                        else
                            x_reg <= x_reg + shift_right(y_reg, iter_counter);
                            y_reg <= y_reg - shift_right(x_reg, iter_counter);
                            z_reg <= z_reg + ATAN_TABLE(iter_counter);
                        end if;
                        
                        iter_counter <= iter_counter + 1;
                    end if;

                when FINISHED =>
                    sin_out <= y_reg;
                    cos_out <= x_reg;
                    done <= '1';
                    state <= IDLE;
            end case;
        end if;
    end process;
end architecture rtl;
----------------------------------------------------------------------------------
-- Company: Personal
-- Engineer: Jose Segura Montes
-- 
-- Create Date: 2026
-- Design Name: FPGA-Accelerated N-DOF Industrial Robot Kinematics
-- Module Name: mult_m - rtl
-- Project Name: ros2-fpga
-- Target Devices: Generic FPGA (Xilinx/Intel)
-- Tool Versions: GHDL / Vivado / Quartus
-- Description: High-performance, parameterizable VHDL-based Direct Kinematics 
--              solver for a N-DOF industrial robot. Implements a 
--              pipelined architecture using Parallel CORDICs (trigonometric 
--              functions) and a custom Matrix Multiplier (4x4) operating under 
--              a robust Qx.y fixed-point arithmetic system.
--
-- Dependencies: robot_config_pkg, htm_pkg
-- 
-- Revision: 13/09/2026
--
-- Copyright (c) 2026 Jose Segura Montes. All rights reserved.
-- This code is licensed under proprietary/open-source terms as applicable.
----------------------------------------------------------------------------------

----------------------------------------------------------------------------------
-- Architecture & Interface Documentation
-- 
-- Entity Name: mult_m
-- Purpose: Implements a pipelined 4x4 matrix multiplication algorithm for computing the product of two homogeneous matrices.
-- Functionality: The module takes two 4x4 matrices as input and computes their product, outputting the resulting matrix.
-- It works using a 5-stage pipeline, where:
--  1. MULT_STAGE: Computes all the products of corresponding elements from the input matrices.
-- 2. SUM_STAGE_1: Sums the first two products for each element of the resulting matrix.
-- 3. SUM_STAGE_2: Continues summing the remaining products for each element.
-- The last row of the resulting matrix is set to [0, 0, 0, 1] to maintain homogeneous coordinates.
--
-- Inputs:
--   * clk      (std_logic)              : System clock driving the internal FSM pipeline.
--   * reset    (std_logic)              : Asynchronous active-high reset signal.
--   * start    (std_logic)              : Single-cycle trigger pulse to initiate computation.
--   * a        (matrix_4x4_in)          : First input matrix (4x4).
--   * b        (matrix_4x4_in)          : Second input matrix (4x4).
--
-- Outputs:
--   * result   (matrix_4x4_out)         : Output matrix (4x4) containing the product of a and b.
--   * done     (std_logic)              : Completion flag asserted for one clock cycle.
--   * done     (std_logic)              : Completion flag asserted for one clock cycle.
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.htm_pkg.all;
use work.robot_config_pkg.all;

entity mult_m is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
        start   : in  std_logic;
        
        a       : in  matrix_4x4_in;
        b       : in  matrix_4x4_in;
        
        result  : out matrix_4x4_out;
        done    : out std_logic
    );
end entity mult_m;

architecture rtl of mult_m is
    -- FSM Types
    type state_type is (IDLE, MULT_STAGE, SUM_STAGE_1, SUM_STAGE_2, FINISHED);
    signal state : state_type := IDLE;
    
    signal a_reg, b_reg : matrix_4x4_in;
    signal result_reg   : matrix_4x4_out;

    -- Intern types for pipelined multiplication and summation
    type array_3d is array (0 to 2, 0 to 3, 0 to 2) of fp_mult_type;
    type array_2d is array (0 to 2, 0 to 3) of fp_mult_type;
    type array_1d is array (0 to 2) of fp_mult_type;

    signal m_reg        : array_3d; 
    signal sum1_reg     : array_2d; 
    signal m3_delay_reg : array_2d; 
    signal t_delay1     : array_1d; 
    signal t_delay2     : array_1d; 

begin
    process(clk, reset)
    begin
        if reset = '1' then
            state <= IDLE;
            done <= '0';
            
            -- Initialize all registers to zero
            for i in 0 to 2 loop
                for j in 0 to 3 loop
                    for k in 0 to 2 loop
                        m_reg(i, j, k) <= (others => '0');
                    end loop;
                    sum1_reg(i, j)     <= (others => '0');
                    m3_delay_reg(i, j) <= (others => '0');
                    result_reg(i, j)   <= (others => '0');
                end loop;
                t_delay1(i) <= (others => '0');
                t_delay2(i) <= (others => '0');
            end loop;
            
        elsif rising_edge(clk) then
            done <= '0'; 

            case state is
                when IDLE =>
                    if start = '1' then
                        a_reg <= a;
                        b_reg <= b;
                        state <= MULT_STAGE;
                    end if;

                when MULT_STAGE =>
                    for i in 0 to 2 loop
                        for j in 0 to 3 loop
                            -- 32-bit * 32-bit = 64-bit
                            m_reg(i, j, 0) <= a_reg(i, 0) * b_reg(0, j);
                            m_reg(i, j, 1) <= a_reg(i, 1) * b_reg(1, j);
                            m_reg(i, j, 2) <= a_reg(i, 2) * b_reg(2, j);
                        end loop;
                        
                        -- Store the shifted value of a_reg(i, 3) for the last column multiplication
                        t_delay1(i) <= shift_left(resize(a_reg(i, 3), TOTAL_WIDTH * 2), FRAC_WIDTH);
                    end loop;
                    
                    state <= SUM_STAGE_1;

                when SUM_STAGE_1 =>
                    for i in 0 to 2 loop
                        for j in 0 to 3 loop
                            sum1_reg(i, j) <= m_reg(i, j, 0) + m_reg(i, j, 1);
                            m3_delay_reg(i, j) <= m_reg(i, j, 2);
                        end loop;
                        t_delay2(i) <= t_delay1(i);
                    end loop;
                    
                    state <= SUM_STAGE_2;

                when SUM_STAGE_2 =>
                    for i in 0 to 2 loop
                        for j in 0 to 2 loop 
                            result_reg(i, j) <= sum1_reg(i, j) + m3_delay_reg(i, j);
                        end loop;
                        
                        result_reg(i, 3) <= sum1_reg(i, 3) + m3_delay_reg(i, 3) + t_delay2(i);
                    end loop;
                    
                    result_reg(3, 0) <= (others => '0');
                    result_reg(3, 1) <= (others => '0');
                    result_reg(3, 2) <= (others => '0');
                    
                    -- (2^30 = 1073741824)
                    result_reg(3, 3) <= to_signed(2**FRAC_WIDTH, TOTAL_WIDTH * 2); 

                    state <= FINISHED;

                when FINISHED =>
                    result <= result_reg;
                    done <= '1';
                    state <= IDLE;
                    
            end case;
        end if;
    end process;
end rtl;
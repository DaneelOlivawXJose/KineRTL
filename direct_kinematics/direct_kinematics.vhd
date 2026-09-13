----------------------------------------------------------------------------------
-- Company: Personal / Open Source Robotics
-- Engineer: Jose Segura Montes
-- 
-- Create Date: 2026
-- Design Name: FPGA-Accelerated N-DOF Industrial Robot Kinematics
-- Module Name: direct_kinematics - rtl
-- Project Name: ros2-fpga
-- Target Devices: Generic FPGA (Xilinx/Intel)
-- Tool Versions: GHDL / Vivado / Quartus
-- Description: High-performance, parameterizable VHDL-based Direct Kinematics 
--              solver for an N-DOF industrial robot. Implements a 
--              pipelined architecture using Parallel CORDICs (trigonometric 
--              functions) and a custom Matrix Multiplier (4x4) operating under 
--              a robust Qx.y fixed-point arithmetic system.
--
-- Dependencies: robot_config_pkg, htm_pkg, mult_m, cordic
-- 
-- Revision: 13/09/2026
--
-- Copyright (c) 2026 Jose Segura Montes. All rights reserved.
-- This code is licensed under proprietary/open-source terms as applicable.
----------------------------------------------------------------------------------

----------------------------------------------------------------------------------
-- Architecture & Interface Documentation
-- 
-- Entity Name: direct_kinematics
-- Purpose: Implements a pipelined direct kinematics solver for an N-DOF industrial robot.
-- Functionality: The module computes the homogeneous transformation matrix for each joint based on the Denavit-Hartenberg parameters and the joint angles.
-- It operates using a multi-stage FSM pipeline:
-- 1. START_CORDIC / CORDIC: Triggers and waits for the parallel CORDIC array to evaluate trigonometric functions (sine and cosine) for each joint angle.
-- 2. MULT: Computes individual link transformation matrices in parallel using DH parameters ($a, d, \alpha$) and CORDIC outputs.
-- 3. MATRIX: Sequentially multiplies and accumulates all individual link matrices using a dedicated pipelined hardware multiplier (`mult_m`) to compute the final end-effector pose.
--
-- Inputs:
--   * clk      (std_logic)             : System clock driving the internal FSM pipeline.
--   * reset    (std_logic)             : Asynchronous active-high reset signal.
--   * start    (std_logic)             : Single-cycle trigger pulse to initiate computation.
--   * theta_in (array_of_fp)           : Input array of joint angles in fixed-point representation.
--
-- Outputs:
--   * t_matrix (matrix_4x4_out)        : Homogeneous transformation matrix (4x4) representing the end-effector pose (double width).
--   * done     (std_logic)             : Completion flag asserted for one clock cycle.
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Package imports for global robot configuration and homogeneous transformation types
use work.robot_config_pkg.all;
use work.htm_pkg.all;

entity direct_kinematics is
    generic (
        ITERATIONS_COORDIC : integer := ITERATIONS_CORDIC; 
        DATA_WIDTH         : integer := TOTAL_WIDTH   
    );
    port (
        clk      : in  std_logic;
        reset    : in  std_logic;
        start    : in  std_logic;
        
        -- Automatically sized based on the Degree of Freedom (DOF) configuration
        theta_in : in array_of_fp(0 to DOF-1);
        
        -- Final 4x4 Homogeneous Transformation Matrix output
        t_matrix : out matrix_4x4_out; 
        done     : out std_logic
    );
end entity direct_kinematics;

architecture rtl of direct_kinematics is
    -- FSM states for managing the execution pipeline
    type state_type is (IDLE, START_CORDIC, CORDIC, MULT, MATRIX, FINISHED);
    signal state : state_type := IDLE;
    
    signal t_matrix_reg : matrix_4x4_in; 
    signal done_reg     : std_logic;
    
    signal start_all_cordics : std_logic;
    signal cordic_done_array : array_of_std_logic(0 to DOF-1);
    signal theta_in_reg      : array_of_fp(0 to DOF-1);
    signal sin_array         : array_of_fp(0 to DOF-1);
    signal cos_array         : array_of_fp(0 to DOF-1);
    
    signal matrix_idx        : integer range 0 to DOF := 0;
    signal mult_busy         : std_logic := '0';
    signal matrix_array_dk   : array_of_matrices(0 to DOF-1);
    
    signal mult_in_a, mult_in_b : matrix_4x4_in;
    signal mult_result          : matrix_4x4_out;
    signal start_mult           : std_logic := '0';
    signal done_mult            : std_logic;

    -- Cycle counter to track CORDIC execution latency deterministically
    signal cordic_counter : integer range 0 to 64 := 0;

    -- Flat debug signals for monitoring global accumulator translation in waveform viewers
    signal dbg_acc_x, dbg_acc_y, dbg_acc_z : fp_type;

    -- Flat debug signals for inspecting active link parameters during matrix assembly
    signal dbg_current_a   : fp_type;
    signal dbg_current_d   : fp_type;
    signal dbg_link_x      : fp_type;
    signal dbg_link_y      : fp_type;
    signal dbg_link_z      : fp_type;

    -- Parameterized Identity Matrix constant used to initialize the accumulation matrix
    constant IDENTITY_MATRIX : matrix_4x4_in := (
        (FP_ONE,  FP_ZERO, FP_ZERO, FP_ZERO),
        (FP_ZERO, FP_ONE,  FP_ZERO, FP_ZERO),
        (FP_ZERO, FP_ZERO, FP_ONE,  FP_ZERO),
        (FP_ZERO, FP_ZERO, FP_ZERO, FP_ONE)
    );
begin
    done <= done_reg;

    -- Concurrent assignments for real-time internal signal monitoring (GTKWave debugging)
    dbg_acc_x <= t_matrix_reg(0, 3);
    dbg_acc_y <= t_matrix_reg(1, 3);
    dbg_acc_z <= t_matrix_reg(2, 3);

    dbg_current_a <= ROBOT_ROM(matrix_idx).a when matrix_idx < DOF else ROBOT_ROM(DOF-1).a;
    dbg_current_d <= ROBOT_ROM(matrix_idx).d when matrix_idx < DOF else ROBOT_ROM(DOF-1).d;

    dbg_link_x    <= matrix_array_dk(matrix_idx)(0, 3) when matrix_idx < DOF else (others => '0');
    dbg_link_y    <= matrix_array_dk(matrix_idx)(1, 3) when matrix_idx < DOF else (others => '0');
    dbg_link_z    <= matrix_array_dk(matrix_idx)(2, 3) when matrix_idx < DOF else (others => '0');

    -- Instantiate parallel CORDIC modules for each robot joint to evaluate sine and cosine concurrently
    GEN_CORDICS: for i in 0 to DOF-1 generate
        CORDIC_INST : entity work.cordic
            port map (
                clk      => clk,
                reset    => reset,
                start    => start_all_cordics, 
                angle_in => theta_in(i),
                sin_out  => sin_array(i),
                cos_out  => cos_array(i),
                done     => cordic_done_array(i)
            );
    end generate GEN_CORDICS;

    -- Instantiate the custom pipelined hardware matrix multiplier core
    MATRIX_MULTIPLIER : entity work.mult_m
        port map (
            clk     => clk,
            reset   => reset,
            start   => start_mult,
            a       => mult_in_a,
            b       => mult_in_b,
            result  => mult_result,
            done    => done_mult
        );

    -- Main sequential FSM controller process
    process(clk, reset)
    begin
        if reset = '1' then
            t_matrix_reg      <= IDENTITY_MATRIX;
            done_reg          <= '0';
            state             <= IDLE;
            start_all_cordics <= '0';
            cordic_counter    <= 0;
            matrix_idx        <= 0;
            mult_busy         <= '0';
            start_mult        <= '0';
        elsif rising_edge(clk) then
            done_reg <= '0';
            
            case state is
                -- Wait for start trigger command from external master controller
                when IDLE =>
                    if start = '1' then
                        start_all_cordics <= '1';
                        state             <= START_CORDIC; 
                        theta_in_reg      <= theta_in;
                        t_matrix_reg      <= IDENTITY_MATRIX; 
                    end if;

                -- Pulse cleanup state for initiating CORDIC parallel execution
                when START_CORDIC =>
                    start_all_cordics <= '0';
                    cordic_counter    <= 0;
                    state             <= CORDIC;
                    
                -- Wait deterministically for all CORDIC pipelines to finish computation
                when CORDIC =>
                    if cordic_counter = ITERATIONS_COORDIC then
                        state <= MULT;
                    else
                        cordic_counter <= cordic_counter + 1;
                    end if;
                    
                -- Build individual Denavit-Hartenberg transformation matrices for each joint using computed trig results
                when MULT =>
                    for i in 0 to DOF-1 loop
                        matrix_array_dk(i)(0,0) <= resize(cos_array(i), TOTAL_WIDTH);
                        matrix_array_dk(i)(1,0) <= resize(sin_array(i), TOTAL_WIDTH);
                        matrix_array_dk(i)(2,0) <= FP_ZERO;
                        matrix_array_dk(i)(3,0) <= FP_ZERO;

                        matrix_array_dk(i)(0,3) <= resize(shift_right(ROBOT_ROM(i).a * cos_array(i), FRAC_WIDTH), TOTAL_WIDTH); 
                        matrix_array_dk(i)(1,3) <= resize(shift_right(ROBOT_ROM(i).a * sin_array(i), FRAC_WIDTH), TOTAL_WIDTH);
                        matrix_array_dk(i)(2,3) <= resize(ROBOT_ROM(i).d, TOTAL_WIDTH);
                        matrix_array_dk(i)(3,3) <= FP_ONE; 

                        -- Configure rotation and twist components based on link alpha parameter configuration
                        case ROBOT_ROM(i).alpha is
                            when A_0 =>
                                matrix_array_dk(i)(0,1) <= resize(-sin_array(i), TOTAL_WIDTH);
                                matrix_array_dk(i)(1,1) <= resize(cos_array(i), TOTAL_WIDTH);
                                matrix_array_dk(i)(2,1) <= FP_ZERO;
                                matrix_array_dk(i)(3,1) <= FP_ZERO;
                                
                                matrix_array_dk(i)(0,2) <= FP_ZERO;
                                matrix_array_dk(i)(1,2) <= FP_ZERO;
                                matrix_array_dk(i)(2,2) <= FP_ONE; 
                                matrix_array_dk(i)(3,2) <= FP_ZERO;

                            when A_90 =>
                                matrix_array_dk(i)(0,1) <= FP_ZERO;
                                matrix_array_dk(i)(1,1) <= FP_ZERO;
                                matrix_array_dk(i)(2,1) <= FP_ONE; 
                                matrix_array_dk(i)(3,1) <= FP_ZERO;
                                
                                matrix_array_dk(i)(0,2) <= sin_array(i);
                                matrix_array_dk(i)(1,2) <= -cos_array(i);
                                matrix_array_dk(i)(2,2) <= FP_ZERO;
                                matrix_array_dk(i)(3,2) <= FP_ZERO;

                            when A_180 =>
                                matrix_array_dk(i)(0,1) <= sin_array(i);
                                matrix_array_dk(i)(1,1) <= -cos_array(i);
                                matrix_array_dk(i)(2,1) <= FP_ZERO;
                                matrix_array_dk(i)(3,1) <= FP_ZERO;
                                
                                matrix_array_dk(i)(0,2) <= FP_ZERO;
                                matrix_array_dk(i)(1,2) <= FP_ZERO;
                                matrix_array_dk(i)(2,2) <= -FP_ONE; 
                                matrix_array_dk(i)(3,2) <= FP_ZERO;

                            when A_MINUS_90 =>
                                matrix_array_dk(i)(0,1) <= FP_ZERO;
                                matrix_array_dk(i)(1,1) <= FP_ZERO;
                                matrix_array_dk(i)(2,1) <= -FP_ONE; 
                                matrix_array_dk(i)(3,1) <= FP_ZERO;
                                
                                matrix_array_dk(i)(0,2) <= -sin_array(i);
                                matrix_array_dk(i)(1,2) <= cos_array(i);
                                matrix_array_dk(i)(2,2) <= FP_ZERO;
                                matrix_array_dk(i)(3,2) <= FP_ZERO;
                        end case;
                    end loop;
                    matrix_idx <= 0;
                    mult_busy  <= '0';  
                    state      <= MATRIX;
                    
                -- Sequentially multiply link transformation matrices using the hardware multiplier block
                when MATRIX =>
                    if matrix_idx = DOF then
                        state <= FINISHED;
                    else
                        if mult_busy = '0' then
                            mult_in_a  <= t_matrix_reg; 
                            mult_in_b  <= matrix_array_dk(matrix_idx);
                            start_mult <= '1';
                            mult_busy  <= '1';
                        else
                            start_mult <= '0';
                            
                            -- Capture multiplier results, applying symmetrical round-to-nearest logic
                            if done_mult = '1' then
                                for r in 0 to 3 loop
                                    for c in 0 to 3 loop
                                        t_matrix_reg(r,c) <= resize(
                                            shift_right(mult_result(r,c) + FP_HALF_MULT, FRAC_WIDTH), 
                                            TOTAL_WIDTH
                                        );
                                    end loop;
                                end loop;
                                
                                matrix_idx <= matrix_idx + 1; 
                                mult_busy  <= '0';            
                            end if;
                        end if;
                    end if;
                
                -- Conclude computation, assert done flag, and output final widened matrix
                when FINISHED =>
                    done_reg <= '1';
                    for r in 0 to 3 loop
                        for c in 0 to 3 loop
                            t_matrix(r,c) <= resize(t_matrix_reg(r,c), TOTAL_WIDTH * 2); 
                        end loop;
                    end loop;
                    state <= IDLE;
            end case;
        end if;
    end process;
end architecture rtl;
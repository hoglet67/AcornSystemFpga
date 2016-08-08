-- Acorn System FPGA for the Papilio Duo
--
-- Copright (c) 2016 David Banks
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- * Redistributions of source code must retain the above copyright notice,
--   this list of conditions and the following disclaimer.
--
-- * Redistributions in synthesized form must reproduce the above copyright
--   notice, this list of conditions and the following disclaimer in the
--   documentation and/or other materials provided with the distribution.
--
-- * Neither the name of the author nor the names of other contributors may
--   be used to endorse or promote products derived from this software without
--   specific prior written agreement from the author.
--
-- * License is granted for non-commercial use only.  A fee may not be charged
--   for redistributions as source code or in synthesized/hardware form without
--   specific prior written agreement from the author.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- Papilio Duo top-level
--
-- (c) 2016 David Banks

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

-- Generic top-level entity for Papilio Duo board
entity acorn_system_duo is
     port (clk_32M00      : in    std_logic;
           ps2_kbd_clk    : in    std_logic;
           ps2_kbd_data   : in    std_logic;
           ps2_mse_clk    : inout std_logic;
           ps2_mse_data   : inout std_logic;
           ERST           : in    std_logic;
           red            : out   std_logic_vector (3 downto 0);
           green          : out   std_logic_vector (3 downto 0);
           blue           : out   std_logic_vector (3 downto 0);
           vsync          : out   std_logic;
           hsync          : out   std_logic;
           audioL         : out   std_logic;
           audioR         : out   std_logic;
           SRAM_nOE       : out   std_logic;
           SRAM_nWE       : out   std_logic;
           SRAM_nCS       : out   std_logic;
           SRAM_A         : out   std_logic_vector (20 downto 0);
           SRAM_D         : inout std_logic_vector (7 downto 0);
           SDMISO         : in    std_logic;
           SDSS           : out   std_logic;
           SDCLK          : out   std_logic;
           SDMOSI         : out   std_logic;
           LED1           : out   std_logic;
           LED2           : out   std_logic;
           ARDUINO_RESET  : out   std_logic;
           SW1            : in    std_logic;
           FLASH_CS       : out   std_logic;                     -- Active low FLASH chip select
           FLASH_SI       : out   std_logic;                     -- Serial output to FLASH chip SI pin
           FLASH_CK       : out   std_logic;                     -- FLASH clock
           FLASH_SO       : in    std_logic;                     -- Serial input from FLASH chip SO pin
           avr_RxD        : in    std_logic;
           avr_TxD        : out   std_logic;
           -- DIP(0) = Video Mode: sRGB (0) / VGA (1)
           -- DIP(1) = VGA Scan Doubler: MIST (0) / RGB2VGA (1)
           -- DIP(2) = System3 (0) / System5 (1)
           -- DIP(3) = Unused
           DIP            : in    std_logic_vector(3 downto 0);
           TEST           : out    std_logic_vector(7 downto 0)
    );
end entity;

architecture rtl of acorn_system_duo is

-------------
-- Signals
-------------

    signal clock_24        : std_logic;
    signal clock_27        : std_logic;
    signal clock_32        : std_logic;
    signal dac_l_in        : std_logic_vector(9 downto 0);
    signal dac_r_in        : std_logic_vector(9 downto 0);
    signal audio_l         : std_logic_vector(15 downto 0);
    signal audio_r         : std_logic_vector(15 downto 0);
    signal audio           : std_logic;
    signal powerup_reset_n : std_logic;
    signal hard_reset_n    : std_logic;
    signal reset_counter   : std_logic_vector(9 downto 0);
    signal RAM_A           : std_logic_vector(18 downto 0);
    signal RAM_Din         : std_logic_vector(7 downto 0);
    signal RAM_Dout        : std_logic_vector(7 downto 0);
    signal RAM_nWE         : std_logic;
    signal RAM_nOE         : std_logic;
    signal RAM_nCS         : std_logic;
    signal vid_mode        : std_logic_vector(3 downto 0);
    signal s5_mode         : std_logic;
    signal s5_mode_1       : std_logic;
    signal s5_mode_2       : std_logic;
    signal caps_led        : std_logic;
    signal shift_led       : std_logic;

-----------------------------------------------
-- Bootstrap ROM Image from SPI FLASH into SRAM
-----------------------------------------------

    -- start address of user data in FLASH as obtained from bitmerge.py
    -- this is safely beyond the end of the bitstream
    constant user_address_s3    : std_logic_vector(23 downto 0) := x"060000";
    constant user_address_s5    : std_logic_vector(23 downto 0) := x"068000";
    signal   user_address       : std_logic_vector(23 downto 0);

    -- lenth of user data in FLASH = 32KB 
    constant user_length   : std_logic_vector(23 downto 0) := x"008000";

    -- high when FLASH is being copied to SRAM, can be used by user as active high reset
    signal bootstrap_busy  : std_logic;

begin

--------------------------------------------------------
-- Acorn System Core
--------------------------------------------------------

    s5_mode        <= DIP(2);
    vid_mode       <= "00" & DIP(1 downto 0);
    acorn_system : entity work.acorn_system_core
    generic map (
        IncludeICEDebugger => true,
        UseT65Core         => false,
        UseAlanDCore       => true
    )
    port map (
        clock_32       => clock_32,
        clock_24       => clock_24,
        clock_27       => clock_27,
        hard_reset_n   => hard_reset_n,
        ps2_kbd_clk    => ps2_kbd_clk,
        ps2_kbd_data   => ps2_kbd_data,
        ps2_mse_clk    => ps2_mse_clk,
        ps2_mse_data   => ps2_mse_data,
        video_red      => red,
        video_green    => green,
        video_blue     => blue,
        video_vsync    => vsync,
        video_hsync    => hsync,
        audio_l        => audio_l,
        audio_r        => audio_r,
        ext_nOE        => RAM_nOE,
        ext_nWE        => RAM_nWE,
        ext_nCS        => RAM_nCS,
        ext_A          => RAM_A,
        ext_Dout       => RAM_Dout,
        ext_Din        => RAM_Din,
        SDMISO         => SDMISO,
        SDSS           => SDSS,
        SDCLK          => SDCLK,
        SDMOSI         => SDMOSI,
        caps_led       => caps_led,
        shift_led      => shift_led,
        vid_mode       => vid_mode,
        avr_RxD        => avr_RxD,
        avr_TxD        => avr_TxD,
        cpu_addr       => open,
        s5_mode        => s5_mode,
        p_spi_ssel     => '0',
        p_spi_sck      => '0',
        p_spi_mosi     => '0',
        p_spi_miso     => open,
        p_irq_b        => open,
        p_nmi_b        => open,
        p_rst_b        => open,
        test           => TEST
    );
    LED1 <= caps_led;
    LED2 <= shift_led;

    SDSS <= '0';
    SDCLK <= '0';
    SDMOSI <= '0';
    
--------------------------------------------------------
-- Clock Generation
--------------------------------------------------------

    inst_dcm1: entity work.dcm1 port map(
        CLKIN_IN  => clk_32M00,
        CLK0_OUT  => clock_32,
        CLKFX_OUT => clock_24
    );

    inst_dcm2: entity work.dcm2 port map (
        CLKIN_IN  => clk_32M00,
        CLKFX_OUT => clock_27
    );

--------------------------------------------------------
-- Power Up Reset Generation
--------------------------------------------------------

    -- Generate a reliable power up reset, as ERST on the Papilio doesn't do this
    -- Also, perform a power up reset if the master/beeb mode switch is changed
    reset_gen : process(clock_32)
    begin
        if rising_edge(clock_32) then
            s5_mode_1 <= s5_mode;
            s5_mode_2 <= s5_mode_1;
            if (s5_mode_1 /= s5_mode_2) then
                reset_counter <= (others => '0');
            elsif (reset_counter(reset_counter'high) = '0') then
                reset_counter <= reset_counter + 1;
            end if;
            powerup_reset_n <= not ERST and reset_counter(reset_counter'high);
        end if;
    end process;

   -- extend the version seen by the core to hold the 6502 reset during bootstrap
   hard_reset_n <= powerup_reset_n and not bootstrap_busy;

--------------------------------------------------------
-- Audio DACs
--------------------------------------------------------
    
    audioL <= '0';
    audioR <= '0';

--------------------------------------------------------
-- Papilio Duo Misc
--------------------------------------------------------

    -- Follow convention for keeping Arduino reset
    ARDUINO_RESET <= SW1;

--------------------------------------------------------
-- BOOTSTRAP SPI FLASH to SRAM
--------------------------------------------------------

    user_address <= user_address_s5 when s5_mode = '1' else user_address_s3;

    inst_bootstrap: entity work.bootstrap
    generic map (
        user_length     => user_length
    )
    port map(
        clock           => clock_32,
        powerup_reset_n => powerup_reset_n,
        bootstrap_busy  => bootstrap_busy,
        user_address    => user_address,
        RAM_nOE         => RAM_nOE,
        RAM_nWE         => RAM_nWE,
        RAM_nCS         => RAM_nCS,
        RAM_A           => RAM_A,
        RAM_Din         => RAM_Din,
        RAM_Dout        => RAM_Dout,
        SRAM_nOE        => SRAM_nOE,
        SRAM_nWE        => SRAM_nWE,
        SRAM_nCS        => SRAM_nCS,
        SRAM_A          => SRAM_A,
        SRAM_D          => SRAM_D,
        FLASH_CS        => FLASH_CS,
        FLASH_SI        => FLASH_SI,
        FLASH_CK        => FLASH_CK,
        FLASH_SO        => FLASH_SO
    );

end architecture;

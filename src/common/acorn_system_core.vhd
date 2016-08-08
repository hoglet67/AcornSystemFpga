-- Acorn System Core, designed to be platform independant
--
-- Copyright (c) 2016 David Banks
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
-- Acorn Micro Core, designed to be platform independant
--
-- (c) 2016 David Banks

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

entity acorn_system_core is
    generic (
        IncludeICEDebugger : boolean := true;
        UseT65Core         : boolean := false;
        UseAlanDCore       : boolean := true
    );
    port (
        -- Clocks
        clock_32       : in    std_logic;
        clock_24       : in    std_logic;
        clock_27       : in    std_logic;

        -- Hard reset (active low)
        hard_reset_n   : in    std_logic;

        -- Keyboard
        ps2_kbd_clk    : in    std_logic;
        ps2_kbd_data   : in    std_logic;

        -- Mouse
        ps2_mse_clk    : inout std_logic;
        ps2_mse_data   : inout std_logic;

        -- Video
        video_red      : out   std_logic_vector (3 downto 0);
        video_green    : out   std_logic_vector (3 downto 0);
        video_blue     : out   std_logic_vector (3 downto 0);
        video_vsync    : out   std_logic;
        video_hsync    : out   std_logic;

        -- Audio
        audio_l        : out   std_logic_vector (15 downto 0);
        audio_r        : out   std_logic_vector (15 downto 0);

        -- External memory (e.g. SRAM and/or FLASH)
        -- 512KB logical address space
        ext_nOE        : out   std_logic;
        ext_nWE        : out   std_logic;
        ext_nCS        : out   std_logic;
        ext_A          : out   std_logic_vector (18 downto 0);
        ext_Dout       : in    std_logic_vector (7 downto 0);
        ext_Din        : out   std_logic_vector (7 downto 0);

        -- SD Card
        SDMISO         : in    std_logic;
        SDSS           : out   std_logic;
        SDCLK          : out   std_logic;
        SDMOSI         : out   std_logic;

        -- KeyBoard LEDs (active high)
        caps_led       : out   std_logic;
        shift_led      : out   std_logic;

        -- Format of Video
        -- Bit 0 selects 15.625KHz SRGB (0) or 31.5KHz VGA (1)
        -- Bit 1 selects Mist (0) or RGB2VGA (1) scan doubler is used for VGA
        -- Bit 2 inverts hsync
        -- Bit 3 inverts vsync
        vid_mode       : in    std_logic_vector(3 downto 0);

        -- ICE T65 Deubgger 57600 baud serial
        avr_RxD        : in    std_logic;
        avr_TxD        : out   std_logic;

        -- Current CPU address, e.g. to drive a hex display
        cpu_addr       : out   std_logic_vector(15 downto 0);

        -- System 5 Mode
        s5_mode        : in    std_logic;

        -- Co Pro SPI - slave interface
        p_spi_ssel     : in    std_logic;
        p_spi_sck      : in    std_logic;
        p_spi_mosi     : in    std_logic;
        p_spi_miso     : out   std_logic;

        -- Co Pro SPI - interrupts/control
        p_irq_b        : out   std_logic;
        p_nmi_b        : out   std_logic;
        p_rst_b        : out   std_logic;

        -- Test outputs
        test           : out   std_logic_vector(7 downto 0)

    );
end entity;

architecture rtl of acorn_system_core is

-------------
-- Signals
-------------

signal reset        :   std_logic;
signal reset_n      :   std_logic;
signal reset_n_out  :   std_logic;
signal clock_avr    :   std_logic;

-- Clock enable counter
-- CPU and video cycles are interleaved.  The CPU runs at 2 MHz (every 16th
-- cycle) and the video subsystem is enabled on every odd cycle.
signal clken_counter    :   unsigned(4 downto 0);
signal cpu_clken        :   std_logic; -- 1 MHz cycles in which the CPU is enabled
signal cpu_clken1       :   std_logic; -- delayed one cycle for BusMonitor
signal vid_clken        :   std_logic;
signal mhz4_clken       :   std_logic; -- Used by 6522
signal mhz2_clken       :   std_logic; -- Used for latching CPU address for clock stretch
signal mhz1_clken       :   std_logic; -- Used by 6522

-- CPU signals
signal cpu_mode         :   std_logic_vector(1 downto 0);
signal cpu_ready        :   std_logic;
signal cpu_abort_n      :   std_logic;
signal cpu_irq_n        :   std_logic;
signal cpu_nmi_n        :   std_logic;
signal cpu_so_n         :   std_logic;
signal cpu_r_nw         :   std_logic;
signal cpu_nr_w         :   std_logic;
signal cpu_sync         :   std_logic;
signal cpu_ef           :   std_logic;
signal cpu_mf           :   std_logic;
signal cpu_xf           :   std_logic;
signal cpu_ml_n         :   std_logic;
signal cpu_vp_n         :   std_logic;
signal cpu_vda          :   std_logic;
signal cpu_vpa          :   std_logic;
signal cpu_a            :   std_logic_vector(23 downto 0);
signal cpu_di           :   std_logic_vector(7 downto 0);
signal cpu_do           :   std_logic_vector(7 downto 0);
signal cpu_addr_us      :   unsigned (15 downto 0);
signal cpu_dout_us      :   unsigned (7 downto 0);

-- Memory enables
signal ram_enable       :   std_logic;
signal rom_enable       :   std_logic;

-- Keyboard signals
signal keyb_break       :   std_logic;
signal keyb_data        :   std_logic_vector(7 downto 0);

-- FDC signals
signal fdc_enable       :   std_logic;
signal fdc_do           :   std_logic_vector(7 downto 0);

-- Common video signals
signal crtc_vsync       :   std_logic;
signal crtc_hsync       :   std_logic;
signal crtc_vsync_n     :   std_logic;
signal crtc_hsync_n     :   std_logic;
signal crtc_r_out       :   std_logic;
signal crtc_g_out       :   std_logic;
signal crtc_b_out       :   std_logic;

-- VDU40 signals
signal vdu40_clken_counter   :   unsigned(1 downto 0);
signal vdu40_clken      :   std_logic;
signal vdu40_do         :   std_logic_vector(7 downto 0);
signal vdu40_vsync      :   std_logic;
signal vdu40_hsync      :   std_logic;
signal vdu40_de         :   std_logic;
signal vdu40_cursor     :   std_logic;
constant vdu40_lpstb    :   std_logic := '0';
signal vdu40_ma         :   std_logic_vector(13 downto 0);
signal vdu40_ra         :   std_logic_vector(4 downto 0);
signal vdu40_r          :   std_logic;
signal vdu40_g          :   std_logic;
signal vdu40_b          :   std_logic;
signal vdu40_enable     :   std_logic;

-- VDU40 Video Ram signals
signal vdu40_ram_enable :   std_logic;
signal vdu40_ram_we     :   std_logic;
signal vdu40_ram_do     :   std_logic_vector(7 downto 0);
signal vdu40_vid_do     :   std_logic_vector(7 downto 0);

-- VDU80 crtc signals
signal vdu80_clken_counter   :   unsigned(15 downto 0);
signal vdu80_clken      :   std_logic;
signal vdu80_do         :   std_logic_vector(7 downto 0);
signal vdu80_vsync      :   std_logic;
signal vdu80_hsync      :   std_logic;
signal vdu80_de         :   std_logic;
signal vdu80_de1        :   std_logic;
signal vdu80_cursor     :   std_logic;
constant vdu80_lpstb    :   std_logic := '0';
signal vdu80_ma         :   std_logic_vector(13 downto 0);
signal vdu80_ra         :   std_logic_vector(4 downto 0);
signal vdu80_sr         :   std_logic_vector(7 downto 0);
signal vdu80_r          :   std_logic;
signal vdu80_g          :   std_logic;
signal vdu80_b          :   std_logic;
signal vdu80_enable     :   std_logic;

-- VDU80 Video Ram signals
signal vdu80_ram_enable :   std_logic;
signal vdu80_ram_we     :   std_logic;
signal vdu80_ram_do     :   std_logic_vector(7 downto 0);
signal vdu80_vid_do     :   std_logic_vector(7 downto 0);
signal vdu80_char_do    :   std_logic_vector(7 downto 0);

-- Scandoubler signals (Mist)
signal rgbi_in          :   std_logic_vector(3 downto 0);
signal vga0_r           :   std_logic_vector(1 downto 0);
signal vga0_g           :   std_logic_vector(1 downto 0);
signal vga0_b           :   std_logic_vector(1 downto 0);
signal vga0_hs          :   std_logic;
signal vga0_vs          :   std_logic;
signal vga0_mode        :   std_logic;

-- Scandoubler signals (RGB2VGA)
signal vga1_r           :   std_logic_vector(1 downto 0);
signal vga1_g           :   std_logic_vector(1 downto 0);
signal vga1_b           :   std_logic_vector(1 downto 0);
signal vga1_hs          :   std_logic;
signal vga1_vs          :   std_logic;
signal vga1_mode        :   std_logic;
signal rgbi_out         :   std_logic_vector(3 downto 0);
signal vsync_int        :   std_logic;
signal hsync_int        :   std_logic;

-- System VIA signals
signal sys_via_do       :   std_logic_vector(7 downto 0);
signal sys_via_do_oe_n  :   std_logic;
signal sys_via_irq_n    :   std_logic;
signal sys_via_ca1_in   :   std_logic := '0';
signal sys_via_ca2_in   :   std_logic := '0';
signal sys_via_ca2_out  :   std_logic;
signal sys_via_ca2_oe_n :   std_logic;
signal sys_via_pa_in    :   std_logic_vector(7 downto 0);
signal sys_via_pa_out   :   std_logic_vector(7 downto 0);
signal sys_via_pa_oe_n  :   std_logic_vector(7 downto 0);
signal sys_via_cb1_in   :   std_logic := '0';
signal sys_via_cb1_out  :   std_logic;
signal sys_via_cb1_oe_n :   std_logic;
signal sys_via_cb2_in   :   std_logic := '0';
signal sys_via_cb2_out  :   std_logic;
signal sys_via_cb2_oe_n :   std_logic;
signal sys_via_pb_in    :   std_logic_vector(7 downto 0);
signal sys_via_pb_out   :   std_logic_vector(7 downto 0);
signal sys_via_pb_oe_n  :   std_logic_vector(7 downto 0);
signal sys_via_do_r     :   std_logic_vector (7 downto 0);
signal sys_via_enable   :   std_logic;

begin

--------------------------------------------------------
-- Reset generation
--------------------------------------------------------

    -- Keyboard and System VIA and Video are by a power up reset signal
    -- Rest of system is reset by all of the above plus keyboard BREAK key
    reset_n <= reset_n_out and hard_reset_n and not keyb_break;
    reset   <= not reset_n;

--------------------------------------------------------
-- Clock enable generation
--------------------------------------------------------

    clk_counter: process(clock_32)
    begin
        if rising_edge(clock_32) then
            clken_counter <= clken_counter + 1;
        end if;
    end process;

    cpu_clken <= not (clken_counter(0) or clken_counter(1) or clken_counter(2) or clken_counter(3)); -- 0/16
    vid_clken <= clken_counter(0);
    mhz4_clken <= clken_counter(0) and clken_counter(1) and clken_counter(2); -- 7/15/23/31
    mhz2_clken <= mhz4_clken and clken_counter(3); -- 15/31
    mhz1_clken <= mhz2_clken and clken_counter(4); -- 31

--------------------------------------------------------
-- 200,005 6502A CPU
--------------------------------------------------------

    GenDebug: if IncludeICEDebugger generate

        core : entity work.MOS6502CpuMonCore
            generic map (
                UseT65Core   => UseT65Core,
                UseAlanDCore => UseAlanDCore
                )
            port map (
                clock_avr    => clock_avr,
                busmon_clk   => clock_32,
                busmon_clken => cpu_clken1,
                cpu_clk      => clock_32,
                cpu_clken    => cpu_clken,
                IRQ_n        => cpu_irq_n,
                NMI_n        => cpu_nmi_n,
                Sync         => cpu_sync,
                Addr         => cpu_a(15 downto 0),
                R_W_n        => cpu_r_nw,
                Din          => cpu_di,
                Dout         => cpu_do,
                SO_n         => cpu_so_n,
                Res_n_in     => reset_n,
                Res_n_out    => reset_n_out,
                Rdy          => cpu_ready,
                trig         => "00",
                avr_RxD      => avr_RxD,
                avr_TxD      => avr_TxD,
                sw1          => '0',
                nsw2         => hard_reset_n,
                led3         => open,
                led6         => open,
                led8         => open,
                tmosi        => open,
                tdin         => open,
                tcclk        => open
                );

        process(clock_32)
        begin
            if rising_edge(clock_32) then
                clock_avr <= not clock_avr;
                cpu_clken1 <= cpu_clken;
            end if;
        end process;

    end generate;

    GenT65Core: if UseT65Core and not IncludeICEDebugger generate
        core : entity work.T65
        port map (
            cpu_mode,
            reset_n,
            cpu_clken,
            clock_32,
            cpu_ready,
            cpu_abort_n,
            cpu_irq_n,
            cpu_nmi_n,
            cpu_so_n,
            cpu_r_nw,
            cpu_sync,
            cpu_ef,
            cpu_mf,
            cpu_xf,
            cpu_ml_n,
            cpu_vp_n,
            cpu_vda,
            cpu_vpa,
            cpu_a,
            cpu_di,
            cpu_do
        );
        reset_n_out <= '1';
        avr_TxD <= avr_RxD;
    end generate;

    GenAlanDCore: if UseAlanDCore and not IncludeICEDebugger generate
        core : entity work.r65c02
        port map (
            reset    => reset_n,
            clk      => clock_32,
            enable   => cpu_clken,
            nmi_n    => cpu_nmi_n,
            irq_n    => cpu_irq_n,
            di       => unsigned(cpu_di),
            do       => cpu_dout_us,
            addr     => cpu_addr_us,
            nwe      => cpu_r_nw,
            sync     => cpu_sync,
            sync_irq => open,
            Regs     => open
        );
        cpu_do <= std_logic_vector(cpu_dout_us);
        cpu_a(15 downto 0) <= std_logic_vector(cpu_addr_us);
        cpu_a(23 downto 16) <= (others => '0');
        reset_n_out <= '1';
        avr_TxD <= avr_RxD;
    end generate;

    -- CPU configuration and fixed signals
    cpu_mode <= "00"; -- 6502
    cpu_ready <= '1';
    cpu_abort_n <= '1';
    cpu_nmi_n <= '1';
    cpu_so_n <= '1';
    cpu_nr_w <= not cpu_r_nw;
    cpu_irq_n <= sys_via_irq_n;

    -- CPU data bus mux and interrupts
    cpu_di <=
        ext_Dout       when ram_enable = '1' or rom_enable = '1' else
        vdu40_ram_do   when vdu40_ram_enable = '1' else
        vdu40_do       when vdu40_enable = '1' else
        vdu80_ram_do   when vdu80_ram_enable = '1' else
        vdu80_do       when vdu80_enable = '1' else
        sys_via_do_r   when sys_via_enable = '1' else
        fdc_do         when fdc_enable = '1' else
        (others => '0');

    -- System VIA
    system_via : entity work.m6522 port map (
        cpu_a(3 downto 0),
        cpu_do,
        sys_via_do,
        sys_via_do_oe_n,
        cpu_r_nw,
        sys_via_enable,
        '0', -- nCS2
        sys_via_irq_n,
        sys_via_ca1_in,
        sys_via_ca2_in,
        sys_via_ca2_out,
        sys_via_ca2_oe_n,
        sys_via_pa_in,
        sys_via_pa_out,
        sys_via_pa_oe_n,
        sys_via_cb1_in,
        sys_via_cb1_out,
        sys_via_cb1_oe_n,
        sys_via_cb2_in,
        sys_via_cb2_out,
        sys_via_cb2_oe_n,
        sys_via_pb_in,
        sys_via_pb_out,
        sys_via_pb_oe_n,
        mhz2_clken,
        hard_reset_n, -- System VIA is reset by power on reset only
        mhz4_clken,
        clock_32
        );

    -- This is needed as in v003 of the 6522 data out is only valid while I_P2_H is asserted
    -- I_P2_H is driven from mhz1_clken
    data_latch: process(clock_32)
    begin
        if rising_edge(clock_32) then
            if (mhz2_clken = '1') then
                sys_via_do_r  <= sys_via_do;
            end if;
        end if;
    end process;

--------------------------------------------------------
-- 200,002 Teletext 40x25 VDU
--------------------------------------------------------

    vdu40_crtc : entity work.mc6845 port map (
        CLOCK  => clock_32,
        CLKEN  => mhz1_clken,
        nRESET => hard_reset_n,
        ENABLE => vdu40_enable,
        R_nW   => cpu_r_nw,
        RS     => cpu_a(0),
        DI     => cpu_do,
        DO     => vdu40_do,
        VSYNC  => vdu40_vsync,
        HSYNC  => vdu40_hsync,
        DE     => vdu40_de,
        CURSOR => vdu40_cursor,
        LPSTB  => vdu40_lpstb,
        MA     => vdu40_ma,
        RA     => vdu40_ra
    );

    vdu40_ram: entity work.VideoRam generic map (WIDTH => 10) port map (
        -- cpu port
        clka  => clock_32,
        wea   => vdu40_ram_we,
        addra => cpu_a(9 downto 0),
        dina  => cpu_do,
        douta => vdu40_ram_do,
        -- video prot
        clkb  => clock_32,
        web   => '0',
        addrb => vdu40_ma(9 downto 0),
        dinb  => (others => '0'),
        doutb => vdu40_vid_do
    );

    teletext : entity work.saa5050 port map (
        CLOCK       => clock_24,
        CLKEN       => vdu40_clken,
        nRESET      => hard_reset_n,
        DI_CLOCK    => clock_32,
        DI_CLKEN    => vid_clken,
        DI          => vdu40_vid_do(6 downto 0),
        GLR         => not vdu40_hsync,
        DEW         => vdu40_vsync,
        CRS         => not vdu40_ra(0),
        LOSE        => vdu40_de,
        R           => vdu40_r,
        G           => vdu40_g,
        B           => vdu40_b,
        Y           => open
    );

    -- 12 MHz clock enable for SAA5050
    vdu40_clk_gen: process(clock_24)
    begin
        if rising_edge(clock_24) then
            vdu40_clken_counter <= vdu40_clken_counter + 1;
            vdu40_clken <= vdu40_clken_counter(0);
        end if;
    end process;

--------------------------------------------------------
-- 200,019 Teletext 80x25 VDU
--------------------------------------------------------

    vdu80_crtc : entity work.mc6845 port map (
        CLOCK  => clock_32,
        CLKEN  => mhz2_clken,
        nRESET => hard_reset_n,
        ENABLE => vdu80_enable,
        R_nW   => cpu_r_nw,
        RS     => cpu_a(0),
        DI     => cpu_do,
        DO     => vdu80_do,
        VSYNC  => vdu80_vsync,
        HSYNC  => vdu80_hsync,
        DE     => vdu80_de,
        CURSOR => vdu80_cursor,
        LPSTB  => vdu80_lpstb,
        MA     => vdu80_ma,
        RA     => vdu80_ra
    );

    vdu80_ram: entity work.VideoRam generic map (WIDTH => 11) port map (
        -- cpu port
        clka  => clock_32,
        wea   => vdu80_ram_we,
        addra => cpu_a(10 downto 0),
        dina  => cpu_do,
        douta => vdu80_ram_do,
        -- video prot
        clkb  => clock_24,
        web   => '0',
        addrb => vdu80_ma(10 downto 0),
        dinb  => (others => '0'),
        doutb => vdu80_vid_do
    );

    vdu80_rom : entity work.CharRom port map (
        CLK  => clock_24,
        ADDR => vdu80_vid_do(6 downto 0) & vdu80_ra(3 downto 0),
        DATA => vdu80_char_do
    );

    vdu80_clk_gen: process(clock_24)
    begin
        if rising_edge(clock_24) then
            if vdu80_clken_counter = "1011" then
                vdu80_clken_counter <= (others => '0');
                vdu80_sr <= vdu80_char_do;
            else
                vdu80_clken_counter <= vdu80_clken_counter + 1;
                if vdu80_clken_counter(0) = '1' then
                    vdu80_sr <= vdu80_sr(6 downto 0) & '0';     
                end if;
            end if;
            if vdu80_clken_counter(0) = '1' then
                vdu80_de1 <= vdu80_de;
            end if;
            if vdu80_de1 = '1' then
                vdu80_r <= vdu80_sr(7) xor vdu80_cursor;
                vdu80_g <= vdu80_sr(7) xor vdu80_cursor;
                vdu80_b <= vdu80_sr(7) xor vdu80_cursor;
            else
                vdu80_r <= '0';
                vdu80_g <= '0';
                vdu80_b <= '0';
            end if;
        end if;
    end process;


--------------------------------------------------------
-- 200,004 FDC
--------------------------------------------------------

fdc_do <= x"00" when cpu_a(7 downto 0) = "00000000" else
          x"04" when cpu_a(7 downto 0) = "00000001" else
          x"AA" when cpu_a(7 downto 0) = "00000100" else
          x"00";

--------------------------------------------------------
-- 200,013 Keyboard
--------------------------------------------------------

caps_led   <= '0';
shift_led  <= '0';
keyb_break <= '0';
keyb_data  <= "11000000";

sys_via_pa_in <= keyb_data;

--------------------------------------------------------
-- Address decoding
--------------------------------------------------------

    rom_enable       <= '1' when cpu_a(15 downto 14) = "11"         else -- 0xC000-0xFFFF
                        '0';

    ram_enable       <= '1' when cpu_a(15 downto 10) = "000000"     else -- 0x0000-0x03FF
                        '1' when cpu_a(15 downto 13) = "001"        else -- 0x2000-0x3FFF
                        '1' when cpu_a(15 downto 14) = "01"         else -- 0x4000-0x7FFF
                        '0';

    vdu40_ram_enable <= '1' when cpu_a(15 downto 10) = "000001"     else -- 0x0400-0x07FF
                        '0';

    vdu40_ram_we     <= vdu40_ram_enable and cpu_nr_w and cpu_clken;

    vdu40_enable      <= '1' when cpu_a(15 downto 8) = "00001000"   else -- 0x0800-0x08FF
                        '0';

    vdu80_ram_enable <= '1' when cpu_a(15 downto 11) = "00010"      else -- 0x1000-0x17FF
                        '0';

    vdu80_ram_we     <= vdu80_ram_enable and cpu_nr_w and cpu_clken;

    vdu80_enable      <= '1' when cpu_a(15 downto 6) = "0001100001" else -- 0x1840-0x187F
                        '0';

    fdc_enable       <= '1' when cpu_a(15 downto 8) = "00001010"    else -- 0x0A00-0x0AFF
                        '0';

    sys_via_enable   <= '1' when cpu_a(15 downto 9) = "0000111"     else -- 0x0E00-0x0FFF
                        '0';
--------------------------------------------------------
-- External RAM
--------------------------------------------------------

    -- SRAM bus
    ext_nCS <= '0';

    -- Synchronous outputs to External Memory

    -- ext_A is 16..0 providing a 128KB address space

    -- 0x00000-0x0FFFF is ROM
    -- 0x00000-0x1FFFF is RAM

    process(clock_32,hard_reset_n)
    begin

        if hard_reset_n = '0' then
            ext_A   <= (others => '0');
            ext_nWE <= '1';
            ext_nOE <= '1';
            ext_Din <= (others => '0');
        elsif rising_edge(clock_32) then
            if rom_enable = '1' then
                ext_A   <= "0000" & cpu_a(14 downto 0);
                ext_nWE <= cpu_r_nw;
                ext_nOE <= not cpu_r_nw;
            elsif ram_enable = '1' then
                ext_A   <= "001" & cpu_a(15 downto 0);
                ext_nWE <= cpu_r_nw;
                ext_nOE <= not cpu_r_nw;
            else
                ext_A   <= (others => '0');
                ext_nWE <= '1';
                ext_nOE <= '1';
            end if;
            ext_Din <= cpu_do;
        end if;
    end process;

-----------------------------------------------
-- Video Mux between VDU 40 and VDU 80
-----------------------------------------------

    crtc_r_out   <= vdu80_r when s5_mode = '1' else vdu40_r;
    crtc_g_out   <= vdu80_g when s5_mode = '1' else vdu40_g;
    crtc_b_out   <= vdu80_b when s5_mode = '1' else vdu40_b;
    crtc_hsync   <= vdu80_hsync when s5_mode = '1' else vdu40_hsync;
    crtc_vsync   <= vdu80_vsync when s5_mode = '1' else vdu40_vsync;

    crtc_hsync_n <= not crtc_hsync;
    crtc_vsync_n <= not crtc_vsync;

-----------------------------------------------
-- Scan Doubler from the MIST project
-----------------------------------------------

    inst_mist_scandoubler: entity work.mist_scandoubler port map (
        clk => clock_32,
        clk_16 => clock_32,
        clk_16_en => vid_clken,
        scanlines => '0',
        hs_in => crtc_hsync_n,
        vs_in => crtc_vsync_n,
        r_in => crtc_r_out,
        g_in => crtc_g_out,
        b_in => crtc_b_out,
        hs_out => vga0_hs,
        vs_out => vga0_vs,
        r_out => vga0_r,
        g_out => vga0_g,
        b_out => vga0_b,
        is15k => open
    );
    vga0_mode <= '1' when vid_mode(0) = '1' and vid_mode(1) = '0' else '0';

-----------------------------------------------
-- Scan Doubler from RGB2VGA project
-----------------------------------------------

    rgbi_in <= crtc_r_out & crtc_g_out & crtc_b_out & '0';

    inst_rgb2vga_scandoubler: entity work.rgb2vga_scandoubler port map (
        clock => clock_32,
        clken => vid_clken,
        clk25 => clock_27,
        rgbi_in => rgbi_in,
        hSync_in => crtc_hsync,
        vSync_in => crtc_vsync,
        rgbi_out => rgbi_out,
        hSync_out => vga1_hs,
        vSync_out => vga1_vs
    );

    vga1_r  <= rgbi_out(3) & rgbi_out(3);
    vga1_g  <= rgbi_out(2) & rgbi_out(2);
    vga1_b  <= rgbi_out(1) & rgbi_out(1);

    vga1_mode <= '1' when vid_mode(0) = '1' and vid_mode(1) = '1' else '0';

-----------------------------------------------
-- RGBHV Multiplexor
-----------------------------------------------

    -- CRTC drives video out (CSYNC on HSYNC output, VSYNC high)
    hsync_int   <= vga0_hs when vga0_mode = '1' else
                    vga1_hs when vga1_mode = '1' else
                    not (crtc_hsync or crtc_vsync);

    vsync_int   <= vga0_vs when vga0_mode = '1' else
                   vga1_vs when vga1_mode = '1' else
                   '1';

    video_hsync <= hsync_int xor vid_mode(2);

    video_vsync <= vsync_int xor vid_mode(3);

    video_red   <= vga0_r(1) & vga0_r(0) & "00" when vga0_mode = '1' else
                   vga1_r(1) & vga1_r(0) & "00" when vga1_mode = '1' else
                   crtc_r_out & crtc_r_out & crtc_r_out & crtc_r_out;

    video_green <= vga0_g(1) & vga0_g(0) & "00" when vga0_mode = '1' else
                   vga1_g(1) & vga1_g(0) & "00" when vga1_mode = '1' else
                   crtc_g_out & crtc_g_out & crtc_g_out & crtc_g_out;

    video_blue  <= vga0_b(1) & vga0_b(0) & "00" when vga0_mode = '1' else
                   vga1_b(1) & vga1_b(0) & "00" when vga1_mode = '1' else
                   crtc_b_out & crtc_b_out & crtc_b_out & crtc_b_out;

    -- Debugging output
    cpu_addr <= cpu_a(15 downto 0);

    -- Test output
    test <= crtc_r_out & crtc_b_out & crtc_g_out & crtc_hsync & crtc_vsync & vdu80_ma(2 downto 0);

end architecture;

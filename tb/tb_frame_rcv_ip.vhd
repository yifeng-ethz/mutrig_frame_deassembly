library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_frame_rcv_ip is
end entity tb_frame_rcv_ip;

architecture sim of tb_frame_rcv_ip is

    constant CLK_PERIOD_CONST      : time := 8 ns;
    constant CTRL_IDLE_CONST       : std_logic_vector(8 downto 0) := "000000001";
    constant CTRL_RUN_PREP_CONST   : std_logic_vector(8 downto 0) := "000000010";
    constant CTRL_SYNC_CONST       : std_logic_vector(8 downto 0) := "000000100";
    constant CTRL_RUNNING_CONST    : std_logic_vector(8 downto 0) := "000001000";
    constant CTRL_TERMINATE_CONST  : std_logic_vector(8 downto 0) := "000010000";
    constant HEADER_BYTE_CONST             : natural := 16#1C#;
    constant ACTIVE_HIT_WORD_CONST         : std_logic_vector(47 downto 0) := x"CAFEBABE1234";
    constant DELAYED_TAIL_HIT_WORD_CONST   : std_logic_vector(47 downto 0) := x"0F1E2D3C4B5A";
    constant TERMINATE_WAIT_MAX_CYCLES     : natural := 4096;
    constant DELAYED_TAIL_GAP_CYCLES_CONST : natural := 64;

    signal asi_rx8b1k_data         : std_logic_vector(8 downto 0) := (others => '0');
    signal asi_rx8b1k_valid        : std_logic := '0';
    signal asi_rx8b1k_error        : std_logic_vector(2 downto 0) := (others => '0');
    signal asi_rx8b1k_channel      : std_logic_vector(3 downto 0) := "0010";
    signal aso_hit_type0_channel   : std_logic_vector(3 downto 0);
    signal aso_hit_type0_startofpacket : std_logic;
    signal aso_hit_type0_endofpacket   : std_logic;
    signal aso_hit_type0_endofrun      : std_logic;
    signal aso_hit_type0_error     : std_logic_vector(2 downto 0);
    signal aso_hit_type0_data      : std_logic_vector(44 downto 0);
    signal aso_hit_type0_valid     : std_logic;
    signal aso_headerinfo_data     : std_logic_vector(41 downto 0);
    signal aso_headerinfo_valid    : std_logic;
    signal aso_headerinfo_channel  : std_logic_vector(3 downto 0);
    signal avs_csr_readdata        : std_logic_vector(31 downto 0);
    signal avs_csr_read            : std_logic := '0';
    signal avs_csr_address         : std_logic_vector(1 downto 0) := (others => '0');
    signal avs_csr_waitrequest     : std_logic;
    signal avs_csr_write           : std_logic := '0';
    signal avs_csr_writedata       : std_logic_vector(31 downto 0) := (others => '0');
    signal asi_ctrl_data           : std_logic_vector(8 downto 0) := (others => '0');
    signal asi_ctrl_valid          : std_logic := '0';
    signal asi_ctrl_ready          : std_logic;
    signal i_rst                   : std_logic := '1';
    signal i_clk                   : std_logic := '0';

    signal hit_count               : integer := 0;
    signal hit_eop_count           : integer := 0;
    signal endofrun_count          : integer := 0;
    signal headerinfo_count        : integer := 0;

    procedure wait_cycles(
        signal clk                 : in std_logic;
        constant cycles            : in natural
    ) is
    begin
        for idx in 1 to cycles loop
            wait until rising_edge(clk);
        end loop;
    end procedure wait_cycles;

    procedure drive_symbol(
        signal clk                 : in  std_logic;
        signal rx_data             : out std_logic_vector(8 downto 0);
        signal rx_valid            : out std_logic;
        signal rx_error            : out std_logic_vector(2 downto 0);
        constant byteisk_value     : in  std_logic;
        constant byte_value        : in  natural
    ) is
    begin
        rx_data                    <= byteisk_value & std_logic_vector(to_unsigned(byte_value, 8));
        rx_valid                   <= '1';
        rx_error                   <= (others => '0');
        wait until rising_edge(clk);
        rx_data                    <= (others => '0');
        rx_valid                   <= '0';
        rx_error                   <= (others => '0');
    end procedure drive_symbol;

    procedure send_ctrl_until_ready(
        signal clk                 : in  std_logic;
        signal ctrl_data           : out std_logic_vector(8 downto 0);
        signal ctrl_valid          : out std_logic;
        signal ctrl_ready          : in  std_logic;
        constant ctrl_word         : in  std_logic_vector(8 downto 0)
    ) is
    begin
        ctrl_data                  <= ctrl_word;
        ctrl_valid                 <= '1';
        loop
            wait until rising_edge(clk);
            exit when ctrl_ready = '1';
        end loop;
        ctrl_valid                 <= '0';
        ctrl_data                  <= (others => '0');
    end procedure send_ctrl_until_ready;

    procedure send_long_frame(
        signal clk                 : in  std_logic;
        signal rx_data             : out std_logic_vector(8 downto 0);
        signal rx_valid            : out std_logic;
        signal rx_error            : out std_logic_vector(2 downto 0);
        constant frame_number      : in  natural;
        constant hit_word          : in  std_logic_vector(47 downto 0)
    ) is
    begin
        drive_symbol(clk, rx_data, rx_valid, rx_error, '1', HEADER_BYTE_CONST);
        drive_symbol(clk, rx_data, rx_valid, rx_error, '0', frame_number / 256);
        drive_symbol(clk, rx_data, rx_valid, rx_error, '0', frame_number mod 256);
        drive_symbol(clk, rx_data, rx_valid, rx_error, '0', 0);
        drive_symbol(clk, rx_data, rx_valid, rx_error, '0', 1);
        for byte_idx in 5 downto 0 loop
            drive_symbol(
                clk      => clk,
                rx_data  => rx_data,
                rx_valid => rx_valid,
                rx_error => rx_error,
                byteisk_value => '0',
                byte_value => to_integer(unsigned(hit_word(byte_idx * 8 + 7 downto byte_idx * 8)))
            );
        end loop;
        drive_symbol(clk, rx_data, rx_valid, rx_error, '0', 0);
        drive_symbol(clk, rx_data, rx_valid, rx_error, '0', 0);
    end procedure send_long_frame;

begin

    i_clk <= not i_clk after CLK_PERIOD_CONST / 2;

    dut : entity work.frame_rcv_ip
        generic map (
            CHANNEL_WIDTH  => 4,
            CSR_ADDR_WIDTH => 2,
            MODE_HALT      => 0,
            DEBUG_LV       => 0
        )
        port map (
            asi_rx8b1k_data         => asi_rx8b1k_data,
            asi_rx8b1k_valid        => asi_rx8b1k_valid,
            asi_rx8b1k_error        => asi_rx8b1k_error,
            asi_rx8b1k_channel      => asi_rx8b1k_channel,
            aso_hit_type0_channel   => aso_hit_type0_channel,
            aso_hit_type0_startofpacket => aso_hit_type0_startofpacket,
            aso_hit_type0_endofpacket   => aso_hit_type0_endofpacket,
            aso_hit_type0_endofrun      => aso_hit_type0_endofrun,
            aso_hit_type0_error     => aso_hit_type0_error,
            aso_hit_type0_data      => aso_hit_type0_data,
            aso_hit_type0_valid     => aso_hit_type0_valid,
            aso_headerinfo_data     => aso_headerinfo_data,
            aso_headerinfo_valid    => aso_headerinfo_valid,
            aso_headerinfo_channel  => aso_headerinfo_channel,
            avs_csr_readdata        => avs_csr_readdata,
            avs_csr_read            => avs_csr_read,
            avs_csr_address         => avs_csr_address,
            avs_csr_waitrequest     => avs_csr_waitrequest,
            avs_csr_write           => avs_csr_write,
            avs_csr_writedata       => avs_csr_writedata,
            asi_ctrl_data           => asi_ctrl_data,
            asi_ctrl_valid          => asi_ctrl_valid,
            asi_ctrl_ready          => asi_ctrl_ready,
            i_rst                   => i_rst,
            i_clk                   => i_clk
        );

    monitor_outputs : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                hit_count           <= 0;
                hit_eop_count       <= 0;
                endofrun_count <= 0;
                headerinfo_count    <= 0;
            else
                if aso_hit_type0_valid = '1' then
                    hit_count <= hit_count + 1;
                    if aso_hit_type0_endofpacket = '1' then
                        hit_eop_count <= hit_eop_count + 1;
                    end if;
                end if;
                if aso_hit_type0_endofrun = '1' then
                    endofrun_count <= endofrun_count + 1;
                end if;
                if aso_headerinfo_valid = '1' then
                    headerinfo_count <= headerinfo_count + 1;
                end if;
            end if;
        end if;
    end process monitor_outputs;

    stim : process
        variable base_hits          : integer;
        variable base_hit_eops      : integer;
        variable base_endofruns     : integer;
        variable base_headerinfo    : integer;
        variable ready_seen         : boolean;
        variable endofrun_seen      : boolean;
        variable active_eop_seen    : boolean;
    begin
        wait_cycles(i_clk, 5);
        i_rst <= '0';
        wait_cycles(i_clk, 2);

        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_RUN_PREP_CONST);
        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_SYNC_CONST);
        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_RUNNING_CONST);
        wait_cycles(i_clk, 2);

        base_hits       := hit_count;
        base_hit_eops   := hit_eop_count;
        base_headerinfo := headerinfo_count;
        send_long_frame(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, 1, x"112233445566");
        wait_cycles(i_clk, 8);
        assert hit_count = base_hits + 1
            report "Expected one parsed hit during RUNNING"
            severity failure;
        assert hit_eop_count = base_hit_eops + 1
            report "Expected one real hit_type0 EOP during RUNNING"
            severity failure;
        assert headerinfo_count = base_headerinfo + 1
            report "Expected one headerinfo pulse during RUNNING"
            severity failure;

        base_endofruns := endofrun_count;
        asi_ctrl_data  <= CTRL_TERMINATE_CONST;
        asi_ctrl_valid <= '1';
        wait_cycles(i_clk, 2);
        asi_ctrl_valid <= '0';
        asi_ctrl_data  <= (others => '0');
        ready_seen     := false;
        for wait_idx in 0 to TERMINATE_WAIT_MAX_CYCLES loop
            wait until rising_edge(i_clk);
            if asi_ctrl_ready = '1' then
                ready_seen := true;
            end if;
            exit when ready_seen and endofrun_count = base_endofruns + 1;
        end loop;
        assert ready_seen
            report "Idle TERMINATING should restore ctrl_ready after the guarded close"
            severity failure;
        assert endofrun_count = base_endofruns + 1
            report "Idle TERMINATING should emit exactly one hit_type0_endofrun pulse"
            severity failure;

        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_IDLE_CONST);
        base_hits       := hit_count;
        base_headerinfo := headerinfo_count;
        base_endofruns  := endofrun_count;
        send_long_frame(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, 2, x"A1B2C3D4E5F6");
        wait_cycles(i_clk, 12);
        assert hit_count = base_hits
            report "IDLE must stay quiescent and refuse new frame parsing"
            severity failure;
        assert headerinfo_count = base_headerinfo
            report "IDLE must not emit headerinfo for a fresh frame"
            severity failure;
        assert endofrun_count = base_endofruns
            report "IDLE must not emit extra hit_type0_endofrun pulses"
            severity failure;

        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_RUN_PREP_CONST);
        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_SYNC_CONST);
        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_RUNNING_CONST);
        wait_cycles(i_clk, 2);

        base_hits        := hit_count;
        base_hit_eops    := hit_eop_count;
        base_headerinfo  := headerinfo_count;
        base_endofruns   := endofrun_count;
        asi_ctrl_data    <= CTRL_TERMINATE_CONST;
        asi_ctrl_valid   <= '1';
        wait_cycles(i_clk, DELAYED_TAIL_GAP_CYCLES_CONST);
        assert endofrun_count = base_endofruns
            report "Delayed terminating tail must not emit hit_type0_endofrun before the final frame arrives"
            severity failure;
        assert asi_ctrl_ready = '0'
            report "Delayed terminating tail must hold ctrl_ready low while waiting for the late final frame"
            severity failure;
        send_long_frame(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, 3, DELAYED_TAIL_HIT_WORD_CONST);

        ready_seen      := false;
        active_eop_seen := false;
        endofrun_seen   := false;
        for wait_idx in 0 to TERMINATE_WAIT_MAX_CYCLES loop
            wait until rising_edge(i_clk);
            if aso_hit_type0_endofpacket = '1' and aso_hit_type0_valid = '1' then
                active_eop_seen := true;
            end if;
            if aso_hit_type0_endofrun = '1' then
                endofrun_seen := true;
            end if;
            if asi_ctrl_ready = '1' then
                ready_seen := true;
                exit when active_eop_seen and endofrun_seen;
            end if;
        end loop;
        asi_ctrl_valid <= '0';
        asi_ctrl_data  <= (others => '0');

        assert ready_seen
            report "Delayed terminating tail should acknowledge only after the late final frame is drained"
            severity failure;
        assert hit_count = base_hits + 1
            report "Delayed terminating tail should still deliver the late final frame hit"
            severity failure;
        assert hit_eop_count = base_hit_eops + 1
            report "Delayed terminating tail should complete with the real frame EOP"
            severity failure;
        assert headerinfo_count = base_headerinfo + 1
            report "Delayed terminating tail should still emit headerinfo for the late final frame"
            severity failure;
        assert endofrun_seen
            report "Delayed terminating tail should emit exactly one hit_type0_endofrun pulse after the late final frame"
            severity failure;
        assert endofrun_count = base_endofruns + 1
            report "Delayed terminating tail should create exactly one hit_type0_endofrun pulse"
            severity failure;

        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_RUN_PREP_CONST);
        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_SYNC_CONST);
        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_RUNNING_CONST);
        wait_cycles(i_clk, 2);

        base_hits        := hit_count;
        base_hit_eops    := hit_eop_count;
        base_endofruns   := endofrun_count;
        drive_symbol(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, '1', HEADER_BYTE_CONST);
        drive_symbol(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, '0', 0);
        drive_symbol(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, '0', 3);
        drive_symbol(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, '0', 0);
        drive_symbol(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, '0', 1);

        asi_ctrl_data  <= CTRL_TERMINATE_CONST;
        asi_ctrl_valid <= '1';

        for byte_idx in 5 downto 0 loop
            drive_symbol(
                clk           => i_clk,
                rx_data       => asi_rx8b1k_data,
                rx_valid      => asi_rx8b1k_valid,
                rx_error      => asi_rx8b1k_error,
                byteisk_value => '0',
                byte_value    => to_integer(unsigned(ACTIVE_HIT_WORD_CONST(byte_idx * 8 + 7 downto byte_idx * 8)))
            );
        end loop;
        drive_symbol(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, '0', 0);
        drive_symbol(i_clk, asi_rx8b1k_data, asi_rx8b1k_valid, asi_rx8b1k_error, '0', 0);

        ready_seen      := false;
        active_eop_seen := false;
        endofrun_seen   := false;
        for wait_idx in 0 to TERMINATE_WAIT_MAX_CYCLES loop
            wait until rising_edge(i_clk);
            if aso_hit_type0_endofpacket = '1' and aso_hit_type0_valid = '1' then
                active_eop_seen := true;
            end if;
            if aso_hit_type0_endofrun = '1' then
                endofrun_seen := true;
            end if;
            if asi_ctrl_ready = '1' then
                ready_seen := true;
                exit when active_eop_seen and endofrun_seen;
            end if;
        end loop;
        assert ready_seen
            report "TERMINATING should acknowledge once the active frame is fully drained"
            severity failure;
        asi_ctrl_valid <= '0';
        asi_ctrl_data  <= (others => '0');

        assert hit_count = base_hits + 1
            report "Active TERMINATING drain should still deliver the in-flight frame hit"
            severity failure;
        assert hit_eop_count = base_hit_eops + 1
            report "Active TERMINATING drain should complete with the real frame EOP"
            severity failure;
        assert endofrun_seen
            report "Active TERMINATING drain should emit a dedicated hit_type0_endofrun pulse after the real frame EOP"
            severity failure;
        assert endofrun_count = base_endofruns + 1
            report "Active frame drain should create exactly one hit_type0_endofrun pulse"
            severity failure;

        send_ctrl_until_ready(i_clk, asi_ctrl_data, asi_ctrl_valid, asi_ctrl_ready, CTRL_IDLE_CONST);
        report "tb_frame_rcv_ip PASSED" severity note;
        finish;
    end process stim;

end architecture sim;

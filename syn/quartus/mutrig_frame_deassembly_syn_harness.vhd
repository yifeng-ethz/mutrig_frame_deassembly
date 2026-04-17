library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mutrig_frame_deassembly_syn_harness is
    port (
        clk125    : in  std_logic;
        reset_n   : in  std_logic;
        probe_out : out std_logic_vector(31 downto 0)
    );
end entity mutrig_frame_deassembly_syn_harness;

architecture rtl of mutrig_frame_deassembly_syn_harness is
    constant CTRL_IDLE_CONST       : std_logic_vector(8 downto 0) := "000000001";
    constant CTRL_RUN_PREP_CONST   : std_logic_vector(8 downto 0) := "000000010";
    constant CTRL_SYNC_CONST       : std_logic_vector(8 downto 0) := "000000100";
    constant CTRL_RUNNING_CONST    : std_logic_vector(8 downto 0) := "000001000";
    constant HEADER_BYTE_CONST     : std_logic_vector(8 downto 0) := '1' & x"1C";
    constant LONG_HIT_WORD_CONST   : std_logic_vector(47 downto 0) := x"112233445566";
    constant EMPTY_PAYLOAD_CONST   : std_logic_vector(47 downto 0) := (others => '0');

    type stim_state_t is (
        ST_RESET_HOLD,
        ST_SEND_PREP,
        ST_SEND_SYNC,
        ST_SEND_RUNNING,
        ST_FRAME_HEADER,
        ST_FRAME_NUMBER_HI,
        ST_FRAME_NUMBER_LO,
        ST_FRAME_FLAGS,
        ST_FRAME_LEN,
        ST_PAYLOAD_5,
        ST_PAYLOAD_4,
        ST_PAYLOAD_3,
        ST_PAYLOAD_2,
        ST_PAYLOAD_1,
        ST_PAYLOAD_0,
        ST_CRC_HI,
        ST_CRC_LO,
        ST_EMPTY_FRAME_FLAGS,
        ST_EMPTY_FRAME_LEN,
        ST_EMPTY_CRC_HI,
        ST_EMPTY_CRC_LO
    );

    signal rst                     : std_logic;
    signal stim_state              : stim_state_t;
    signal reset_hold_ctr          : unsigned(4 downto 0);
    signal frame_counter           : unsigned(15 downto 0);
    signal signature               : std_logic_vector(31 downto 0);
    signal do_empty_frame          : std_logic;

    signal asi_rx8b1k_data         : std_logic_vector(8 downto 0);
    signal asi_rx8b1k_valid        : std_logic;
    signal asi_rx8b1k_error        : std_logic_vector(2 downto 0);
    signal asi_rx8b1k_channel      : std_logic_vector(3 downto 0);

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
    signal avs_csr_read            : std_logic;
    signal avs_csr_address         : std_logic_vector(1 downto 0);
    signal avs_csr_waitrequest     : std_logic;
    signal avs_csr_write           : std_logic;
    signal avs_csr_writedata       : std_logic_vector(31 downto 0);

    signal asi_ctrl_data           : std_logic_vector(8 downto 0);
    signal asi_ctrl_valid          : std_logic;
    signal asi_ctrl_ready          : std_logic;

    function pack_symbol(is_k: std_logic; value: std_logic_vector(7 downto 0)) return std_logic_vector is
    begin
        return is_k & value;
    end function;
begin
    rst                   <= not reset_n;
    probe_out             <= signature;
    avs_csr_read          <= '0';
    avs_csr_address       <= (others => '0');
    avs_csr_write         <= '0';
    avs_csr_writedata     <= (others => '0');

    process (clk125, rst)
        variable payload_word : std_logic_vector(47 downto 0);
    begin
        if rst = '1' then
            stim_state               <= ST_RESET_HOLD;
            reset_hold_ctr           <= (others => '0');
            frame_counter            <= (others => '0');
            signature                <= (others => '0');
            do_empty_frame           <= '0';
            asi_rx8b1k_data          <= (others => '0');
            asi_rx8b1k_valid         <= '0';
            asi_rx8b1k_error         <= (others => '0');
            asi_rx8b1k_channel       <= (others => '0');
            asi_ctrl_data            <= CTRL_IDLE_CONST;
            asi_ctrl_valid           <= '0';
        elsif rising_edge(clk125) then
            asi_rx8b1k_data          <= (others => '0');
            asi_rx8b1k_valid         <= '0';
            asi_rx8b1k_error         <= (others => '0');
            asi_rx8b1k_channel       <= std_logic_vector(frame_counter(3 downto 0));
            asi_ctrl_data            <= CTRL_IDLE_CONST;
            asi_ctrl_valid           <= '0';

            payload_word := LONG_HIT_WORD_CONST xor (x"00000000" & std_logic_vector(frame_counter));

            case stim_state is
                when ST_RESET_HOLD =>
                    if reset_hold_ctr = to_unsigned(15, reset_hold_ctr'length) then
                        stim_state <= ST_SEND_PREP;
                    else
                        reset_hold_ctr <= reset_hold_ctr + 1;
                    end if;

                when ST_SEND_PREP =>
                    asi_ctrl_data  <= CTRL_RUN_PREP_CONST;
                    asi_ctrl_valid <= '1';
                    if asi_ctrl_ready = '1' then
                        stim_state <= ST_SEND_SYNC;
                    end if;

                when ST_SEND_SYNC =>
                    asi_ctrl_data  <= CTRL_SYNC_CONST;
                    asi_ctrl_valid <= '1';
                    if asi_ctrl_ready = '1' then
                        stim_state <= ST_SEND_RUNNING;
                    end if;

                when ST_SEND_RUNNING =>
                    asi_ctrl_data  <= CTRL_RUNNING_CONST;
                    asi_ctrl_valid <= '1';
                    if asi_ctrl_ready = '1' then
                        stim_state <= ST_FRAME_HEADER;
                    end if;

                when ST_FRAME_HEADER =>
                    asi_rx8b1k_data  <= HEADER_BYTE_CONST;
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_FRAME_NUMBER_HI;

                when ST_FRAME_NUMBER_HI =>
                    asi_rx8b1k_data  <= pack_symbol('0', std_logic_vector(frame_counter(15 downto 8)));
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_FRAME_NUMBER_LO;

                when ST_FRAME_NUMBER_LO =>
                    asi_rx8b1k_data  <= pack_symbol('0', std_logic_vector(frame_counter(7 downto 0)));
                    asi_rx8b1k_valid <= '1';
                    if do_empty_frame = '1' then
                        stim_state   <= ST_EMPTY_FRAME_FLAGS;
                    else
                        stim_state   <= ST_FRAME_FLAGS;
                    end if;

                when ST_FRAME_FLAGS =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"00");
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_FRAME_LEN;

                when ST_FRAME_LEN =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"01");
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_PAYLOAD_5;

                when ST_PAYLOAD_5 =>
                    asi_rx8b1k_data  <= pack_symbol('0', payload_word(47 downto 40));
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_PAYLOAD_4;

                when ST_PAYLOAD_4 =>
                    asi_rx8b1k_data  <= pack_symbol('0', payload_word(39 downto 32));
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_PAYLOAD_3;

                when ST_PAYLOAD_3 =>
                    asi_rx8b1k_data  <= pack_symbol('0', payload_word(31 downto 24));
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_PAYLOAD_2;

                when ST_PAYLOAD_2 =>
                    asi_rx8b1k_data  <= pack_symbol('0', payload_word(23 downto 16));
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_PAYLOAD_1;

                when ST_PAYLOAD_1 =>
                    asi_rx8b1k_data  <= pack_symbol('0', payload_word(15 downto 8));
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_PAYLOAD_0;

                when ST_PAYLOAD_0 =>
                    asi_rx8b1k_data  <= pack_symbol('0', payload_word(7 downto 0));
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_CRC_HI;

                when ST_CRC_HI =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"00");
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_CRC_LO;

                when ST_CRC_LO =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"00");
                    asi_rx8b1k_valid <= '1';
                    frame_counter    <= frame_counter + 1;
                    do_empty_frame   <= not do_empty_frame;
                    stim_state       <= ST_FRAME_HEADER;

                when ST_EMPTY_FRAME_FLAGS =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"00");
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_EMPTY_FRAME_LEN;

                when ST_EMPTY_FRAME_LEN =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"00");
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_EMPTY_CRC_HI;

                when ST_EMPTY_CRC_HI =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"00");
                    asi_rx8b1k_valid <= '1';
                    stim_state       <= ST_EMPTY_CRC_LO;

                when ST_EMPTY_CRC_LO =>
                    asi_rx8b1k_data  <= pack_symbol('0', x"00");
                    asi_rx8b1k_valid <= '1';
                    frame_counter    <= frame_counter + 1;
                    do_empty_frame   <= not do_empty_frame;
                    stim_state       <= ST_FRAME_HEADER;
            end case;

            if aso_hit_type0_valid = '1' then
                signature <= signature xor aso_hit_type0_data(31 downto 0);
            elsif aso_headerinfo_valid = '1' then
                signature <= signature xor aso_headerinfo_data(31 downto 0);
            else
                signature(3 downto 0) <= signature(3 downto 0) xor aso_hit_type0_channel;
                signature(7 downto 4) <= signature(7 downto 4) xor aso_headerinfo_channel;
                signature(8)          <= signature(8) xor aso_hit_type0_startofpacket;
                signature(9)          <= signature(9) xor aso_hit_type0_endofpacket;
                signature(10)         <= signature(10) xor aso_hit_type0_endofrun;
                signature(13 downto 11) <= signature(13 downto 11) xor aso_hit_type0_error;
                signature(14)         <= signature(14) xor aso_hit_type0_valid;
                signature(15)         <= signature(15) xor aso_headerinfo_valid;
                signature(31 downto 16) <= signature(31 downto 16) xor std_logic_vector(frame_counter);
            end if;
        end if;
    end process;

    u_dut : entity work.frame_rcv_ip
        generic map (
            CHANNEL_WIDTH  => 4,
            CSR_ADDR_WIDTH => 2,
            MODE_HALT      => 0,
            DEBUG_LV       => 0
        )
        port map (
            asi_rx8b1k_data             => asi_rx8b1k_data,
            asi_rx8b1k_valid            => asi_rx8b1k_valid,
            asi_rx8b1k_error            => asi_rx8b1k_error,
            asi_rx8b1k_channel          => asi_rx8b1k_channel,
            aso_hit_type0_channel       => aso_hit_type0_channel,
            aso_hit_type0_startofpacket => aso_hit_type0_startofpacket,
            aso_hit_type0_endofpacket   => aso_hit_type0_endofpacket,
            aso_hit_type0_endofrun      => aso_hit_type0_endofrun,
            aso_hit_type0_error         => aso_hit_type0_error,
            aso_hit_type0_data          => aso_hit_type0_data,
            aso_hit_type0_valid         => aso_hit_type0_valid,
            aso_headerinfo_data         => aso_headerinfo_data,
            aso_headerinfo_valid        => aso_headerinfo_valid,
            aso_headerinfo_channel      => aso_headerinfo_channel,
            avs_csr_readdata            => avs_csr_readdata,
            avs_csr_read                => avs_csr_read,
            avs_csr_address             => avs_csr_address,
            avs_csr_waitrequest         => avs_csr_waitrequest,
            avs_csr_write               => avs_csr_write,
            avs_csr_writedata           => avs_csr_writedata,
            asi_ctrl_data               => asi_ctrl_data,
            asi_ctrl_valid              => asi_ctrl_valid,
            asi_ctrl_ready              => asi_ctrl_ready,
            i_rst                       => rst,
            i_clk                       => clk125
        );
end architecture rtl;

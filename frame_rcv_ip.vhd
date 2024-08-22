----------------------------------------------------------------------------------
-- Company:
-- Engineer: Huangshan Chen (chen@kip.uni-heidelberg.de)
-- Engineer2: Yifeng Wang (yifenwan@phys.ethz.ch)
--
-- Create Date: 16.05.2016 (IP packeged on 20.02.2024)
-- Design Name:
-- Module Name: frame receiver module
-- Project Name:
-- Target Devices: Arria V (Intel FPGA)
-- Tool versions: Quartus 18.1, Platform Designer 16.1
-- Description:
--
-- taking data from deserialzer, and form event data (YW: see more comments below for ip packaging)
--
-- Dependencies: crc16_calc.vhd (submodule)
--
-- Revision:
-- Revision 0.01 - File Created
-- Revision 1.00 - adpated to MuTRiG frame structure
-- Revision 1.10 - KB: Preparations for datapath_v2: Use record types, conversion of data depending on hit type
-- Revision 1.20 - Yifeng Wang: Adaptations/wrapping for IP packaging. (support stream data and avmm config)
-- Revision 1.21 - YW: correct the sop/eop to the first/last hit valid cycles. 

-- Additional Comments:
-- IP wrapper layer: 
-- Input: lvds 8b1k data, forms packet of hits. 
-- Output: 1) mutrig packets in avst frame, 2) header info in avst.
-- Controlled by: 1) avst enable signal (standard RUN_SEQUENCE) 2) avmm CSR (debug mode to bypass/ignore errors/exceptions)
----------------------------------------------------------------------------------

-- ================ synthsizer configuration =================== 		
-- altera vhdl_input_version vhdl_2008
-- ============================================================= 

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.math_real.log2;
use IEEE.math_real.ceil;
use ieee.std_logic_arith.conv_std_logic_vector;

entity frame_rcv_ip is
generic (
	DEBUG_BYPASS_ERROR		: natural := 0; 
	SIMULATION_EN			: natural := 0; 
    ASIC_ID                 : natural := 0 -- channel index dummy generic, the output channel is set by the input avst channel 
);
port(
	-- input stream of 8b1k from lvds receiver
	asi_rx8b1k_data					: in  std_logic_vector(8 downto 0);
	asi_rx8b1k_valid				: in  std_logic;
	asi_rx8b1k_error				: in  std_logic_vector(2 downto 0); 
		-- disperr & decerr & lane_training_fail;
	asi_rx8b1k_channel 				: in  std_logic_vector(3 downto 0); -- the output channel is set by it
	 
    -- output stream of hits
	aso_hit_type0_channel			: out std_logic_vector(3 downto 0); -- for asic 0-15
	aso_hit_type0_startofpacket		: out std_logic;
	aso_hit_type0_endofpacket		: out std_logic;
	aso_hit_type0_error				: out std_logic_vector(2 downto 0); 
	-- frame_corrupt & crcerr & hiterr
		-- crcerr available at "eop"
		-- hiterr available at "valid"
	aso_hit_type0_data				: out std_logic_vector(44 downto 0); -- valid is a seperate signal below
	aso_hit_type0_valid 			: out std_logic;

	-- output stream of header
    -- header information
	aso_headerinfo_data				: out std_logic_vector(41 downto 0);
	aso_headerinfo_valid			: out std_logic;
	aso_headerinfo_channel			: out std_logic_vector(3 downto 0);

	-- avmm-slave
	-- control and status registers
	avs_csr_readdata				: out std_logic_vector(31 downto 0);
	avs_csr_read					: in  std_logic;
	avs_csr_address					: in  std_logic_vector(0 downto 0);
	avs_csr_waitrequest				: out std_logic;
	avs_csr_byteenable				: in  std_logic_vector(3 downto 0);
	avs_csr_write					: in  std_logic;
	avs_csr_writedata				: in  std_logic_vector(31 downto 0);
	 
	-- Flow control inside system: it was controlled by the `ctrl_start' signal. Now, it should be controlled by the 
	-- ready signal from the upstream, which is pipelined within the datapath. The ready allowance 
	-- of the downstream module must be higher than the length of a frame. (no, ready allowance is only up to 8) 
	-- So, a frame can be flushed through with the RUN_END incidence. Otherwise, it is also possible
	-- to do a backpressure before the channel mux. Then, the sorter (itself is a buffer), will consume 
	-- the from the backpressure buffer after the mux. 
	
	-- input stream of control signal (enable)
	-- this signal is time critical and must be synchronzed for all datapath modules
	asi_ctrl_data			: in  std_logic_vector(8 downto 0); 
	asi_ctrl_valid			: in  std_logic;
	asi_ctrl_ready			: out std_logic;
	
	-- reset / clock / enable
    i_rst                   : in std_logic; -- async reset assertion, sync reset release
    i_clk                   : in std_logic -- clock should match the input lvds parallel clock

);
end entity frame_rcv_ip;

architecture rtl of frame_rcv_ip is

	-- Hit type (long) before sorter, before TCC division
	-- ==================================================================
    constant len_hit_presort : natural := 4 + 5 + 15 + 5 + 15 + 1 + 1; -- 46 bits (1bit for valid)
    type t_hit_presort is record
        asic            : std_logic_vector(3 downto 0);  --ASIC ID
        channel         : std_logic_vector(4 downto 0);  --Channel number
        T_CC            : std_logic_vector(14 downto 0); --T-Trigger coarse time value (1.6ns)
        T_Fine          : std_logic_vector(4 downto 0);  --T-Trigger fine time value
        E_CC            : std_logic_vector(14 downto 0); --Energy coarse time value (in units of 1.6ns)
        E_Flag          : std_logic;                     --E-Flag valid flag
        valid           : std_logic;                     --data word valid flag
		hiterr			: std_logic;
    end record;
    --type t_v_hit_presort is array (natural range <>) of t_hit_presort;
	
	-- 8b/10b symbols
	constant k28_0					: integer := 16#1C#; -- header
	constant k28_4					: integer := 16#9C#; -- trailer
	constant k28_5					: integer := 16#BC#; -- filler
	constant c_header				: std_logic_vector := std_logic_vector(to_unsigned(k28_0, 8));

    constant EVENT_DATA_WIDTH        : positive := 48;
    constant N_BYTES_PER_WORD        : positive := 6;
    constant N_BYTES_PER_WORD_SHORT  : positive := 3;

    signal s_o_word                     : std_logic_vector(EVENT_DATA_WIDTH-1 downto 0);
    signal p_word, n_word               : std_logic_vector(N_BYTES_PER_WORD*8 -1 downto 0);
    signal p_word_extra, n_word_extra   : std_logic_vector(3 downto 0);
    signal p_new_word, n_new_word       : std_logic;

    -- CRC specific signals
    signal s_crc_result                     : std_logic_vector(15 downto 0);
    -- start of a new CRC calculation
    signal p_crc_din_valid, n_crc_din_valid : std_logic;
    signal p_crc_rst, n_crc_rst             : std_logic;
    signal p_crc_err_count, n_crc_err_count : std_logic_vector(31 downto 0);
    signal p_crc_error, n_crc_error         : std_logic;
    signal p_end_of_frame, n_end_of_frame   : std_logic;
    signal s_o_frame_info                   : std_logic_vector(15 downto 0);

    -- frame information signals
    -- indicates a new frame
    signal p_new_frame, n_new_frame         : std_logic;
    signal p_frame_number, n_frame_number   : std_logic_vector(15 downto 0);
    signal p_word_cnt, n_word_cnt           : unsigned(9 downto 0);
    signal p_frame_len, n_frame_len         : std_logic_vector(9 downto 0);
     -- frame_flags : | i_SC_gen_idle_sig | i_SC_fast_mode | i_SC_prbs_debug | i_SC_single_prbs | p_fifo_full | '0' |
    signal p_frame_flags, n_frame_flags     : std_logic_vector(5 downto 0);
    signal p_frame_info_ready, n_frame_info_ready, enable : std_logic;
	signal hit_err							: std_logic;
	signal sop_comb,eop_comb				: std_logic;

    -- FSM states
    type FSM_states is (
        FS_IDLE,
        FS_FRAME_COUNTER,
        FS_EVENT_COUNTER,
        FS_UNPACK,
        FS_UNPACK_EXTRA,
        FS_CRC_CALC,
        FS_CRC_CHECK,
        FS_ERROR
    );
    signal p_state, n_state : FSM_states;
    signal p_state_wait_cnt, n_state_wait_cnt : natural range 7 downto 0; --cnt to wait withiin state


    signal p_txflag_isShort : std_logic;
    signal p_txflag_isCEC   : std_logic;
	 
	-- some depreciated io port signal are downgraded as internal signal
	-- data from lvds receiver
	signal i_data                  		: std_logic_vector(7 downto 0);
	signal i_byteisk               		: std_logic;
	
	-- header information
	signal o_hits						: t_hit_presort;
	signal o_frame_number				: std_logic_vector(15 downto 0);
	signal o_frame_info					: std_logic_vector(15 downto 0);
	signal o_frame_info_ready			: std_logic;
	signal o_new_frame					: std_logic;
	signal o_busy						: std_logic;
	 
	-- trailer information
	signal o_end_of_frame          		: std_logic;
	signal o_crc_error             		: std_logic;
	signal o_crc_err_count         		: std_logic_vector(31 downto 0);
	signal o_cec_flag              		: std_logic;
	signal o_cec_data              		: std_logic_vector(12*32-1 downto 0);
	
	-- control signal
	signal receiver_go						: std_logic; -- register record the setting, set by avst/avmm. 
	signal receiver_force_go				: std_logic;
	
	signal n_frame_info_ready_d1		: std_logic;
	signal aso_headerinfo_valid_comb	: std_logic;
	
	-- avmm csr
	type csr_t is record
		error_cnt		: std_logic_vector(31 downto 0);
		status			: std_logic_vector(7 downto 0); -- bit[5:0]: current frame flag
		control			: std_logic_vector(7 downto 0); -- bit 0: set to run(1)/set to stop(0)
		-- "running" is generate new frame, "idle" is finishing current frame and do not generate new frame
		-- "set to run" is enable, "set to stop" is enable_n (do not generate new frame). 
		-- exception to be implemented
	end record;
	signal csr 		: csr_t;
	
	
	-- run control mgmt
	type run_state_t is (IDLE, RUN_PREPARE, SYNC, RUNNING, TERMINATING, LINK_TEST, SYNC_TEST, RESET, OUT_OF_DAQ, ERROR);
	signal run_state_cmd				: run_state_t;
	 
begin


	-- some mapping of the new avalon signal to the internal old signals
	proc_input_wrapper_comb : process (all)
	-- The halt condition for the frame rcv fsm is [data=BC,byteisk=1].
	-- Halt the fsm if the input is not valid, which should not happen unless the LVDS receiver is not in RUNNING_OK state.
	-- This prevents it from parsing corrupted random digital lvds noise as hits. 
	-- For DEBUG: check asi_rx8b1k_error(0) for lane_training_fail, if the valid goes low.
	begin
		if (asi_rx8b1k_valid = '1') then
			i_data				<= asi_rx8b1k_data(7 downto 0);
			i_byteisk			<= asi_rx8b1k_data(8);
		else 
			i_data				<= x"BC";
			i_byteisk			<= '1';
		end if;
	end process proc_input_wrapper_comb;
	
	proc_output_header_info_comb : process (all)
	begin
		if (n_frame_info_ready_d1 = '0') then
			aso_headerinfo_valid_comb		<= n_frame_info_ready_d1 xor n_frame_info_ready;
		else
			aso_headerinfo_valid_comb		<= '0';
		end if;
	end process proc_output_header_info_comb;
	
	proc_output_header_info : process (i_clk,i_rst)
	begin
		if (i_rst = '1' ) then 
			aso_headerinfo_valid				<= '0';
		elsif rising_edge(i_clk) then
			aso_headerinfo_data(5 downto 0)		<= n_frame_flags;
			aso_headerinfo_data(15 downto 6)	<= n_frame_len;
			aso_headerinfo_data(25 downto 16)	<= std_logic_vector(n_word_cnt);
			aso_headerinfo_data(41 downto 26)	<= n_frame_number;
			aso_headerinfo_valid				<= aso_headerinfo_valid_comb;
			n_frame_info_ready_d1				<= n_frame_info_ready;
			aso_headerinfo_channel				<= asi_rx8b1k_channel;
		end if;
	end process proc_output_header_info;
	

	
	proc_avmm_slave_csr : process (i_clk,i_rst)
	-- avalon memory-mapped interface for accessing the control and status registers
	-- address map:
	-- 		0: control and status (go, frame flag, TBD)
	-- 		1: frame with crc error counter
	-- 		2: total frame counter (TODO)
	begin
		if (i_rst = '1' ) then 
			csr.control		<= (0 => '1', others => '0');
			csr.status		<= (others => '0');
			csr.error_cnt	<= (others => '0');
			avs_csr_waitrequest		<= '1';
		elsif rising_edge(i_clk) then
			-- default
			avs_csr_readdata			<= (others => '0');
			-- logic
			if (avs_csr_read = '1') then
				avs_csr_waitrequest		<= '0';
				case (to_integer(unsigned(avs_csr_address))) is 
					when 0 =>
						avs_csr_readdata(31 downto 24)		<= csr.status;
						avs_csr_readdata(7 downto 0)		<= csr.control;
					when 1 =>
						avs_csr_readdata					<= csr.error_cnt;
					when others =>
				end case;
			elsif (avs_csr_write = '1') then
				avs_csr_waitrequest		<= '0';
				case (to_integer(unsigned(avs_csr_address))) is
					when 0 =>
						csr.control		<= avs_csr_writedata(7 downto 0); -- address 1 (errcnt) cannot be written
					when 1 =>
						csr.error_cnt	<= (others => '0');
					when others =>
				end case;	
			else -- idle, update values to/from agent
				avs_csr_waitrequest		<= '1';
				csr.error_cnt		<= o_crc_err_count; -- continuously update crc error count
				if (n_frame_info_ready = '1') then -- update frame flag 
					csr.status(5 downto 0)		<= n_frame_flags;
				end if;
				csr.status(7 downto 6)		<= (others => '0'); -- TBD
			end if;
		end if;

	end process proc_avmm_slave_csr;
	

	
	proc_enable_ctrl : process (i_clk,i_rst)
	-- avst and avmm both can control the enable of main state machine.
	-- avmm can mask.
	-- avst gives enable at start of run
	begin
		if (i_rst = '1' ) then 
			enable			<= '0'; -- default is disable
		elsif rising_edge(i_clk) then
			if (receiver_force_go = '1') then -- need to ignore mask during idle, for monitoring rate.
				enable 			<= '1';
			elsif (receiver_go = '1') then -- allow to go in running state.
				if (csr.control(0) = '1') then   -- not masked
					enable			<= '1';
				else
					enable			<= '0';
				end if;
			else 
				enable			<= '0';
			end if;
		end if;
	end process proc_enable_ctrl;
	
	
	proc_output_pkt_hits_comb : process(all)
	begin
		
		-- For missing eop (sop-...-sop-eop), this means frame corrupted. Old frame should be discarded
		-- by the FIFO before mux. The ts_remapper can still use the current frame ID to infer the correction
		-- because it is just a pipeline following the frame assembler (no flow control is needed)
		
		-- For missing sop (...-eop-sop-eop), this also means frame corrupted. This frame will not be loaded in the FIFO 
		-- before mux, but user has the option to bypass this discard. 
		
		-- The packet scheduling and flow control should be handled before the mux. 
		-- Corrupted frame can be discarded or kept/flaged depending on the user settings.
		
		-- The mux will grant packet as soon as the eop is seen for good frame. 
		-- The mux will not use packet schedule to minimize the latency across channels.
		-- The channel with showahead of smallest TS will be granted first. (pre-sort)?			
		aso_hit_type0_channel					<= asi_rx8b1k_channel;
		aso_hit_type0_data(44 downto 41)		<= o_hits.asic;
		aso_hit_type0_data(40 downto 36)		<= o_hits.channel;
		aso_hit_type0_data(35 downto 21)		<= o_hits.T_CC;
		aso_hit_type0_data(20 downto 16)		<= o_hits.T_Fine;
		aso_hit_type0_data(15 downto 1)			<= o_hits.E_CC;
		aso_hit_type0_data(0)					<= o_hits.E_Flag;
		aso_hit_type0_valid						<= o_hits.valid;
		
	end process ;
	
	
	
	
	
	
	
	--mutrig3 frame tx flags:
	--0b000 Long event transmission
	--0b001 PRBS transmission, single event
	--0b010 PRBS transmission, saturating link
	--0b100 Short event transmission
	--0b110 Channel event counter data trans
    p_txflag_isShort <= '1' when p_frame_flags(4 downto 2) = "100" else '0';
    p_txflag_isCEC   <= '1' when p_frame_flags(4 downto 2) = "101" else '0';

    o_new_frame     <= p_new_frame;
    o_frame_number  <= p_frame_number;
    o_end_of_frame  <= p_end_of_frame;

    -- the latching of the frame_info is done outside
    -- the frame_flags information will be usefull if it's updated in the beginning of the frame, for the prbs_checker
    o_frame_info    <= p_frame_flags & p_frame_len;
    o_crc_err_count <= p_crc_err_count;
    o_crc_error     <= p_crc_error;

    o_frame_info_ready  <= p_frame_info_ready;


    --assemble record for hit data stream
    -- taking care of the long event word and short event word
    -- if 48-bit word, send out s_o_word
    -- if 27-bit word, the data are at highest 27 bits;
    -- re-arrange the bit:
    --   put the E_flag ( s_o_word(48-27) ) to bit(0)
    --   fill the bit( 48-27 downto 1) with '0'
    --bits not carried in record: T_bad_hit, E_bad_hit, E_fine, E_flag
    o_hits.asic    <= aso_hit_type0_channel;
    o_hits.channel <= s_o_word(47 downto 43);
    o_hits.T_CC    <= s_o_word(41 downto 27);
    o_hits.T_Fine  <= s_o_word(26 downto 22);
    o_hits.E_CC    <= s_o_word(20 downto 6) when p_frame_flags(4) = '0' else (others => '0');
    o_hits.E_Flag  <= s_o_word(21) when p_frame_flags(4) = '0' else s_o_word(0);
    o_hits.valid   <= p_new_word;

    u_crc16 : entity work.crc16_calc
    port map (
        i_clk       => i_clk,
        i_rst       => n_crc_rst,
        i_d_valid   => n_crc_din_valid,
        i_din       => i_data,
        o_crc_reg   => s_crc_result,
        o_crc_8     => open--,
    );

    syn : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                p_state             <= FS_IDLE;
                p_crc_err_count     <= (others => '0');
                p_frame_number      <= (others => '0');
            else
                p_state             <= n_state;
                p_state_wait_cnt    <= n_state_wait_cnt;

                p_word              <= n_word;
                p_word_extra        <= n_word_extra;
                p_new_word          <= n_new_word;

                p_new_frame         <= n_new_frame;
                p_frame_number      <= n_frame_number;
                p_frame_len         <= n_frame_len;
                p_frame_flags       <= n_frame_flags;

                p_word_cnt          <= n_word_cnt;
                p_crc_err_count     <= n_crc_err_count;
                p_crc_din_valid     <= n_crc_din_valid;
                p_crc_rst           <= n_crc_rst;
                p_crc_error         <= n_crc_error;
                p_end_of_frame      <= n_end_of_frame;
                p_frame_info_ready  <= n_frame_info_ready;
				-- SOP/EOP generation
                if n_new_word = '1' then
                    s_o_word <= n_word; -- latch the new word (maybe a bit unnecessary to choose a specific time point)
					if (to_integer(n_word_cnt) = 1) then -- sop
						aso_hit_type0_startofpacket <= '1';
					else
						aso_hit_type0_startofpacket <= '0';
					end if;
					if (to_integer(n_word_cnt) = to_integer(unsigned(n_frame_len))) then -- eop
						aso_hit_type0_endofpacket	<= '1';
					else
						aso_hit_type0_endofpacket	<= '0';
					end if;
				else
					aso_hit_type0_startofpacket <= '0';
					aso_hit_type0_endofpacket	<= '0';
                end if;
            end if;
			-- derive the hit error
			if (p_state = FS_UNPACK or p_state = FS_UNPACK_EXTRA) then -- in receiving hits state
				if (n_new_word = '1') then
					aso_hit_type0_error(0) 		<= hit_err; -- latch the hit error based on any byte error
					hit_err						<= '0'; -- unset hit error
				end if;
				if (asi_rx8b1k_error(2) = '1' or asi_rx8b1k_error(1) = '1') then -- any byte error within the hit
					hit_err				<= '1'; -- set hit error
				end if;
			else
				aso_hit_type0_error(0) 			<= '0';
				hit_err							<= '0';
			end if;
			-- derive lane_training error, span the whole frame
			aso_hit_type0_error(2)		<= asi_rx8b1k_error(0);
			-- derive the crc error, TODO: fix it to the eop/last hit
			aso_hit_type0_error(1)		<= n_crc_error;
			
        end if;
    end process;
	

	


    comb : process(
        i_data, i_byteisk,
        p_state, p_state_wait_cnt, p_crc_err_count, p_crc_din_valid, p_word_cnt, p_new_frame, p_frame_number, p_frame_len,
        n_word_cnt, s_crc_result, p_word, p_word_extra, p_crc_rst, enable, p_frame_flags, p_txflag_isShort
    ) -- !!!!!!!!! MISSING SENSITIVITY LIST "p_txflag_isShort" TODO change to all
    begin
        --DEFAULT SIGNAL ASSIGNMENTS
        o_busy              <= '1';
        n_state             <= p_state;
        n_state_wait_cnt    <= p_state_wait_cnt;

        n_word              <= p_word;
        n_word_extra        <= p_word_extra;
        n_new_word          <= '0';
        n_word_cnt          <= p_word_cnt;

        n_crc_error         <= '0';
        n_crc_err_count     <= p_crc_err_count;
        n_crc_din_valid     <= p_crc_din_valid;
        n_crc_rst           <= p_crc_rst;
        n_end_of_frame      <= '0';

        n_new_frame         <= '0';
        n_frame_len         <= p_frame_len;
        n_frame_number      <= p_frame_number;
        n_frame_flags       <= p_frame_flags;
        n_frame_info_ready  <= '0';
		
		sop_comb			<= '0';
		eop_comb			<= '0';

        if ( i_byteisk = '1' and i_data = x"BC" ) then
            --
        else
            case p_state is
            when FS_IDLE =>
				n_frame_info_ready	<= '0';
                o_busy           <= '0';
                --Initialize the frame len meta information
                n_frame_len      <= (others => '0');
                n_frame_flags    <= (others => '0');
                n_word_cnt       <= (others => '0');

                n_state_wait_cnt <= 0;
                n_crc_din_valid  <= '0';
                n_crc_rst        <= '1';
                n_new_frame      <= '0';

                --state transition
                -- detect frame_header, go to FS_FRAME_COUNTER
                if ( enable = '1' and i_byteisk = '1' and i_data = c_header ) then -- start condition
                    n_new_frame         <= '1'; -- flag the sof for 1 cycle
                    n_state             <= FS_FRAME_COUNTER;
                    n_state_wait_cnt    <= 2;
                end if;

            when FS_FRAME_COUNTER => -- 2 cycles
                n_crc_din_valid <= '1'; -- start the crc engine
                n_crc_rst       <= '0'; -- deassert crc engine reset
                n_frame_number(p_state_wait_cnt*8-1 downto (p_state_wait_cnt-1)*8) <= i_data;
                n_state_wait_cnt <= p_state_wait_cnt - 1;
                if ( p_state_wait_cnt = 1 ) then
                    n_state             <= FS_EVENT_COUNTER;
                    n_state_wait_cnt    <= 2;
                end if;

            when FS_EVENT_COUNTER => -- 2 cycles
                n_state_wait_cnt <= p_state_wait_cnt - 1;
                if ( p_state_wait_cnt = 2 ) then
                    n_frame_flags           <= i_data(7 downto 2);
                    n_frame_len(9 downto 8) <= i_data(1 downto 0);
                elsif ( p_state_wait_cnt = 1 ) then
                    n_frame_len(7 downto 0) <= i_data;
                    n_frame_info_ready      <= '1';
                    -- indicate the start of the new frame
                    if ( p_frame_len(9 downto 8) = "00" and i_data = std_logic_vector(to_unsigned(0,8)) ) then
                        n_state             <= FS_CRC_CALC; -- no hit in this frame
                        n_state_wait_cnt    <= 2;
                    else -- there are hits in this frame
                        n_state <= FS_UNPACK;
						-- derive the hit mode from frame flag which determines op mode of this state machine
                        if ( p_txflag_isShort = '0' ) then 
                            n_state_wait_cnt <= N_BYTES_PER_WORD;
                        else -- interleaving/bang-bang between UNPACK and UNPACK_EXTRA 
                            n_state_wait_cnt <= N_BYTES_PER_WORD_SHORT;
                            n_word           <= (others => '0');
                        end if;
                    end if;
                end if;

            when FS_UNPACK => -- 3 cycles
                 -- {normal mode}
                if ( p_txflag_isShort = '0' ) then
					-- collect 6 bytes data of all hits
                    n_word(p_state_wait_cnt*8-1 downto (p_state_wait_cnt-1)*8) <= i_data;
                    n_state_wait_cnt        <= p_state_wait_cnt - 1;
                    if ( p_state_wait_cnt = 1 ) then
						-- assert the hit is valid
                        n_new_word          <= '1';
                        n_word_cnt          <= p_word_cnt + 1;
                        n_state_wait_cnt    <= N_BYTES_PER_WORD;
                        if ( p_word_cnt = unsigned(p_frame_len(9 downto 0)) - 1 ) then -- exit condition
                            n_state         <= FS_CRC_CALC;
                            n_state_wait_cnt<= 2;
							n_word_cnt          <= p_word_cnt + 1; -- YW: need for gen eop
                        end if;
                    end if;
                 -- {fast mode}
				 -- bang-bang between UNPACK_EXTRA and UNPACK state for even and odd number of hits in short mode
                else
                    n_state_wait_cnt <= p_state_wait_cnt - 1;
                    if ( p_word_cnt(0) = '0' ) then -- current is even number of hits
                        -- collect 3 bytes here
						n_word(n_word'high-(N_BYTES_PER_WORD_SHORT-p_state_wait_cnt)*8 downto n_word'high-(N_BYTES_PER_WORD_SHORT-p_state_wait_cnt+1)*8+1) <= i_data;
                        -- collect the half byte in UNPACK_EXTRA
						if ( p_state_wait_cnt = 1 ) then
                            n_state <= FS_UNPACK_EXTRA; -- go to UNPACK_EXTRA
                        end if;
                    else -- current is odd number of hits, just coming back from UNPACK_EXTRA
                        if ( p_state_wait_cnt = N_BYTES_PER_WORD_SHORT ) then -- coming back from UNPACK_EXTRA state
                            n_word(n_word'high downto n_word'high-3) <= p_word_extra; -- receive 4 bit extra (half-byte) for odd hit, MSB
                        end if;
						-- collect 3 bytes for the odd hit
                        n_word(n_word'high-(N_BYTES_PER_WORD_SHORT-p_state_wait_cnt)*8-4 downto n_word'high-(N_BYTES_PER_WORD_SHORT-p_state_wait_cnt+1)*8-3) <= i_data; -- collect bytes to establish a hit
                        if ( p_state_wait_cnt = 1 ) then -- assert new word to indicate a hit valid when byte counter is at 1
                            n_new_word <= '1';
                            if ( p_word_cnt = unsigned(p_frame_len) - 1 ) then -- exit condition: last hit of this frame
                                n_state             <= FS_CRC_CALC;
                                n_state_wait_cnt    <= 2;
								n_word_cnt          <= p_word_cnt + 1; -- YW: need for gen eop
                            else
                                n_word_cnt          <= p_word_cnt + 1;
                                n_state_wait_cnt    <= N_BYTES_PER_WORD_SHORT;
                            end if;
                        end if;
                    end if;
                end if;

            when FS_UNPACK_EXTRA => -- 1 cycle
                if ( p_word_cnt(0) = '0' ) then -- current is even number of hits (bull-shitting)
					-- collect the half byte to the past even hit, LSB
                    n_word(n_word'high-N_BYTES_PER_WORD_SHORT*8 downto n_word'high-N_BYTES_PER_WORD_SHORT*8-3) <= i_data(7 downto 4);
					-- pass the half byte for the coming odd hit, MSB
                    n_word_extra    <= i_data(3 downto 0);
					-- assert for the even hit
                    n_new_word      <= '1';
                    if ( p_word_cnt = unsigned(p_frame_len) - 1 ) then -- exit condition: last hit of this frame
                        n_state             <= FS_CRC_CALC;
                        n_state_wait_cnt    <= 2;
						n_word_cnt          <= p_word_cnt + 1; -- YW: need for gen eop
                    else
                        n_state             <= FS_UNPACK; -- bang-bang
                        n_word_cnt          <= p_word_cnt + 1;
                        n_state_wait_cnt    <= N_BYTES_PER_WORD_SHORT;
                    end if;
                end if;

            when FS_CRC_CALC => -- 2 cycles
                n_state_wait_cnt <= p_state_wait_cnt -1;
                if ( p_state_wait_cnt = 1 ) then
        --  if i_byteisk = '1' and i_data = c_trailer(31 downto 24) then
                    n_state <= FS_CRC_CHECK;
        --   n_new_frame <= '0';
        --   n_state_wait_cnt <= 2;
        -- else
        --   n_state <= FS_IDLE;
        --   n_crc_err_count <= std_logic_vector(unsigned(p_crc_err_count)+1);
        -- end if;
                end if;

            when FS_CRC_CHECK => -- 1 cycle
                n_crc_din_valid <= '0'; -- stop the crc engine 
                n_crc_rst       <= '1'; -- reset the crc engine
                n_end_of_frame  <= '1'; -- flag eof for 1 cycle
        --      n_state_wait_cnt <= p_state_wait_cnt -1;
        --      if p_state_wait_cnt = 1 then
                if s_crc_result /= X"7FF2" then  -- CORRECT magic number
                --if s_crc_result /= X"FFFF" then  -- WRONG result, to test if crc works
                    n_crc_err_count <= std_logic_vector(unsigned(p_crc_err_count)+1);
                    n_crc_error <= '1';
                end if;
                n_state <= FS_IDLE;
        --  end if;

            when others => null;

        end case;
        end if;
    end process;
	
	proc_run_control_mgmt_agent : process (i_clk,i_rst)
	-- In mu3e run control system, each feb has a run control management host which runs in reset clock domain, while other IPs must feature
	-- run control management agent which listens the run state command to capture the transition.
	-- The state transition are only ack by the agent for as little as 1 cycle, but the host must assert the valid until all ack by the agents are received,
	-- during transitioning period. 
	-- The host should record the timestamps (clock cycle and phase) difference between the run command signal is received by its lvds_rx and 
	-- agents' ready signal. This should ensure all agents are running at the same time, despite there is phase uncertainty between the clocks, which 
	-- might results in 1 clock cycle difference and should be compensated offline. 
	begin
		if (i_rst = '1') then 
			receiver_force_go		<= '0';
			receiver_go				<= '0';
			run_state_cmd			<= IDLE; -- after power-on, we can moniter the rate
		elsif (rising_edge(i_clk)) then 
			-- valid 
			if (asi_ctrl_valid = '1') then 
				-- get payload of run control as run command 
				case asi_ctrl_data is 
					when "000000001" =>
						run_state_cmd		<= IDLE;
					when "000000010" => 
						run_state_cmd		<= RUN_PREPARE;
					when "000000100" =>
						run_state_cmd		<= SYNC;
					when "000001000" =>
						run_state_cmd		<= RUNNING;
					when "000010000" =>
						run_state_cmd		<= TERMINATING;
					when "000100000" => 
						run_state_cmd		<= LINK_TEST;
					when "001000000" =>
						run_state_cmd		<= SYNC_TEST;
					when "010000000" =>
						run_state_cmd		<= RESET;
					when "100000000" =>
						run_state_cmd		<= OUT_OF_DAQ;
					when others =>
						run_state_cmd		<= ERROR;
				end case;
			else 
				run_state_cmd		<= run_state_cmd;
			end if;
			-- ready
				asi_ctrl_ready		<= '1';
			
			-- run control mgmt fsm 
			case run_state_cmd is 
				when IDLE => -- must generate frame, ignore mask
					receiver_force_go		<= '1';
					receiver_go				<= '0';
				when RUN_PREPARE =>
					receiver_force_go		<= '0';
					receiver_go				<= '0';
				when SYNC => 
					receiver_force_go		<= '0';
					receiver_go				<= '0';
				when RUNNING => -- may generate frame, subject to mask
					receiver_force_go		<= '0';
					receiver_go				<= '1';
				when TERMINATING => 
					receiver_force_go		<= '0';
					receiver_go				<= '0'; -- stop generation new frame, but finish current frame
				when others =>
					receiver_force_go		<= '0';
					receiver_go				<= '0';
			end case;
		
		
		
		end if;
	

	
	end process;
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

end architecture;

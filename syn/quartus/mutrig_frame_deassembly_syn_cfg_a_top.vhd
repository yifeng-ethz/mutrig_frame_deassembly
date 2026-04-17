library ieee;
use ieee.std_logic_1164.all;

entity mutrig_frame_deassembly_syn_cfg_a_top is
    port (
        clk125    : in  std_logic;
        reset_n   : in  std_logic;
        probe_out : out std_logic_vector(31 downto 0)
    );
end entity mutrig_frame_deassembly_syn_cfg_a_top;

architecture rtl of mutrig_frame_deassembly_syn_cfg_a_top is
begin
    u_harness : entity work.mutrig_frame_deassembly_syn_harness
        port map (
            clk125    => clk125,
            reset_n   => reset_n,
            probe_out => probe_out
        );
end architecture rtl;
